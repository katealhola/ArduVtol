/*
  SITL handling

  This simulates the APM1 hardware sufficiently for the APM code to
  think it is running on real hardware

  Andrew Tridgell November 2011
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <math.h>
#include <APM_RC.h>
#include <wiring.h>
#include <AP_PeriodicProcess.h>
#include <AP_TimerProcess.h>
#include <SITL.h>
#include <avr/interrupt.h>
#include "sitl_adc.h"
#include "sitl_rc.h"
#include "desktop.h"
#include "util.h"

#define SIMIN_PORT 5501
#define RCOUT_PORT 5502

static int sitl_fd;
struct sockaddr_in rcout_addr;
#ifndef __CYGWIN__
static pid_t parent_pid;
#endif
struct ADC_UDR2 UDR2;
struct RC_ICR4 ICR4;
extern AP_TimerProcess timer_scheduler;
extern Arduino_Mega_ISR_Registry isr_registry;
extern SITL sitl;

static uint32_t update_count;


/*
  setup a SITL FDM listening UDP port
 */
static void setup_fdm(void)
{
	int one=1, ret;
	struct sockaddr_in sockaddr;

	memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
	sockaddr.sin_len = sizeof(sockaddr);
#endif
	sockaddr.sin_port = htons(SIMIN_PORT);
	sockaddr.sin_family = AF_INET;

	sitl_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sitl_fd == -1) {
		fprintf(stderr, "SITL: socket failed - %s\n", strerror(errno));
		exit(1);
	}

	/* we want to be able to re-use ports quickly */
	setsockopt(sitl_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

	ret = bind(sitl_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
	if (ret == -1) {
		fprintf(stderr, "SITL: bind failed on port %u - %s\n",
			(unsigned)ntohs(sockaddr.sin_port), strerror(errno));
		exit(1);
	}

	set_nonblocking(sitl_fd);
}

/*
  check for a SITL FDM packet
 */
static void sitl_fdm_input(void)
{
	ssize_t size;
	struct pwm_packet {
		uint16_t pwm[8];
	};
	union {
		struct sitl_fdm fg_pkt;
		struct pwm_packet pwm_pkt;
	} d;

	size = recv(sitl_fd, &d, sizeof(d), MSG_DONTWAIT);
	switch (size) {
	case 132:
		static uint32_t last_report;
		static uint32_t count;

		if (d.fg_pkt.magic != 0x4c56414e) {
			printf("Bad FDM packet - magic=0x%08x\n", d.fg_pkt.magic);
			return;
		}

		if (d.fg_pkt.latitude == 0 ||
		    d.fg_pkt.longitude == 0 ||
		    d.fg_pkt.altitude <= 0) {
			// garbage input
			return;
		}

		sitl.state = d.fg_pkt;
		update_count++;

		count++;
		if (millis() - last_report > 1000) {
			//printf("SIM %u FPS\n", count);
			count = 0;
			last_report = millis();
		}
		break;

	case 16: {
		// a packet giving the receiver PWM inputs
		uint8_t i;
		for (i=0; i<8; i++) {
			// setup the ICR4 register for the RC channel
			// inputs
			if (d.pwm_pkt.pwm[i] != 0) {
				ICR4.set(i, d.pwm_pkt.pwm[i]);
			}
		}
		break;
	}
	}

}

// used for noise generation in the ADC code
bool sitl_motors_on;

/*
  send RC outputs to simulator
 */
static void sitl_simulator_output(void)
{
	static uint32_t last_update;
	struct {
		uint16_t pwm[11];
		uint16_t speed, direction, turbulance;
	} control;
	/* this maps the registers used for PWM outputs. The RC
	 * driver updates these whenever it wants the channel output
	 * to change */
	uint16_t *reg[11] = { &OCR5B, &OCR5C, &OCR1B, &OCR1C,
			      &OCR4C, &OCR4B, &OCR3C, &OCR3B,
			      &OCR5A, &OCR1A, &OCR3A };
	uint8_t i;

	if (last_update == 0) {
		for (i=0; i<11; i++) {
			(*reg[i]) = 1000*2;
		}
		if (desktop_state.vehicle == ArduPlane) {
			(*reg[0]) = (*reg[1]) = (*reg[3]) = 1500*2;
			(*reg[7]) = 1800*2;
		}
		if (desktop_state.vehicle == APMrover2) {
			(*reg[0]) = (*reg[1]) = (*reg[2]) = (*reg[3]) = 1500*2;
			(*reg[7]) = 1800*2;
		}
	}

	// output at chosen framerate
	if (last_update != 0 && millis() - last_update < 1000/desktop_state.framerate) {
		return;
	}
	last_update = millis();

	for (i=0; i<11; i++) {
		if (*reg[i] == 0xFFFF) {
			control.pwm[i] = 0;
		} else {
			control.pwm[i] = (*reg[i])/2;
		}
	}

	if (desktop_state.vehicle == ArduPlane) {
		// add in engine multiplier
		if (control.pwm[2] > 1000) {
			control.pwm[2] = ((control.pwm[2]-1000) * sitl.engine_mul) + 1000;
			if (control.pwm[2] > 2000) control.pwm[2] = 2000;
		}
		sitl_motors_on = ((control.pwm[2]-1000)/1000.0) > 0;
	} else if (desktop_state.vehicle == APMrover2) {
		// add in engine multiplier
		if (control.pwm[2] != 1500) {
			control.pwm[2] = ((control.pwm[2]-1500) * sitl.engine_mul) + 1500;
			if (control.pwm[2] > 2000) control.pwm[2] = 2000;
			if (control.pwm[2] < 1000) control.pwm[2] = 1000;
		}
		sitl_motors_on = ((control.pwm[2]-1500)/500.0) != 0;
	} else {
		sitl_motors_on = false;
		for (i=0; i<4; i++) {
			if ((control.pwm[i]-1000)/1000.0 > 0) {
				sitl_motors_on = true;
			}
		}
	}

	// setup wind control
	control.speed      = sitl.wind_speed * 100;
	float direction = sitl.wind_direction;
	if (direction < 0) {
		direction += 360;
	}
	control.direction  = direction * 100;
	control.turbulance = sitl.wind_turbulance * 100;

	// zero the wind for the first 15s to allow pitot calibration
	if (millis() < 15000) {
		control.speed = 0;
	}

	sendto(sitl_fd, (void*)&control, sizeof(control), MSG_DONTWAIT, (const sockaddr *)&rcout_addr, sizeof(rcout_addr));
}

/*
  timer called at 1kHz
 */
static void timer_handler(int signum)
{
	static uint32_t last_update_count;
	static bool in_timer;

	if (in_timer || _interrupts_are_blocked()) {
		return;
	}
	uint8_t oldSREG = SREG;
	cli();

	in_timer = true;

#ifndef __CYGWIN__
	/* make sure we die if our parent dies */
	if (kill(parent_pid, 0) != 0) {
		exit(1);
	}
#else
    
	static uint16_t count = 0;
	static uint32_t last_report;
        
    	count++;
	if (millis() - last_report > 1000) {
		printf("TH %u cps\n", count);
		count = 0;
		last_report = millis();
	}
#endif

	/* check for packet from flight sim */
	sitl_fdm_input();

	// trigger RC input
	if (isr_registry._registry[ISR_REGISTRY_TIMER4_CAPT]) {
		isr_registry._registry[ISR_REGISTRY_TIMER4_CAPT]();
	}

	// send RC output to flight sim
	sitl_simulator_output();

	if (update_count == 0) {
		sitl_update_gps(0, 0, 0, 0, 0, false);
		timer_scheduler.run();
		SREG = oldSREG;
		in_timer = false;
		return;
	}

	if (update_count == last_update_count) {
		timer_scheduler.run();
		SREG = oldSREG;
		in_timer = false;
		return;
	}
	last_update_count = update_count;

	sitl_update_gps(sitl.state.latitude, sitl.state.longitude,
			sitl.state.altitude,
			sitl.state.speedN, sitl.state.speedE, !sitl.gps_disable);
	sitl_update_adc(sitl.state.rollDeg, sitl.state.pitchDeg, sitl.state.yawDeg,
			sitl.state.rollRate, sitl.state.pitchRate, sitl.state.yawRate,
			sitl.state.xAccel, sitl.state.yAccel, sitl.state.zAccel,
			sitl.state.airspeed);
	sitl_update_barometer(sitl.state.altitude);
	sitl_update_compass(sitl.state.rollDeg, sitl.state.pitchDeg, sitl.state.heading);

	// clear the ADC conversion flag,
	// so the ADC code doesn't get stuck
	ADCSRA &= ~_BV(ADSC);

	// trigger all APM timers. We do this last as it can re-enable
	// interrupts, which can lead to recursion
	timer_scheduler.run();

	SREG = oldSREG;
	in_timer = false;
}


/*
  setup a timer used to prod the ISRs
 */
static void setup_timer(void)
{
	struct itimerval it;
	struct sigaction act;

	act.sa_handler = timer_handler;
        act.sa_flags = SA_RESTART|SA_NODEFER;
        sigemptyset(&act.sa_mask);
        sigaddset(&act.sa_mask, SIGALRM);
        sigaction(SIGALRM, &act, NULL);

	it.it_interval.tv_sec = 0;
	it.it_interval.tv_usec = 1000; // 1KHz
	it.it_value = it.it_interval;

	setitimer(ITIMER_REAL, &it, NULL);
}


/*
  setup for SITL handling
 */
void sitl_setup(void)
{
#ifndef __CYGWIN__
	parent_pid = getppid();
#endif

	rcout_addr.sin_family = AF_INET;
	rcout_addr.sin_port = htons(RCOUT_PORT);
	inet_pton(AF_INET, "127.0.0.1", &rcout_addr.sin_addr);

	setup_timer();
	setup_fdm();
	sitl_setup_adc();
	printf("Starting SITL input\n");

	// setup some initial values
	sitl_update_barometer(desktop_state.initial_height);
	sitl_update_adc(0, 0, 0, 0, 0, 0, 0, 0, -9.8, 0);
	sitl_update_compass(0, 0, 0);
	sitl_update_gps(0, 0, 0, 0, 0, false);
}


