/* 
 *  Squeezelite - lightweight headless squeezebox emulator
 *
 *  (c) Adrian Smith 2012-2015, triode1@btinternet.com
 *      Ralph Irving 2015-2017, ralph_irving@hotmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// SPI DAC support - will push data to the DAC driver

#include "squeezelite.h"

#if DSPAUDIO



static log_level loglevel;

static bool running = true;

extern struct outputstate output;
extern struct buffer *outputbuf;

#define LOCK   mutex_lock(outputbuf->mutex)
#define UNLOCK mutex_unlock(outputbuf->mutex)

extern u8_t *silencebuf;
#if DSD
extern u8_t *silencebuf_dsd;
#endif

void list_devices(void) {
  //todo: get the DAC details and output to log

// 	if ((err = Pa_Initialize()) != paNoError) {
// 		LOG_WARN("error initialising port audio: %s", Pa_GetErrorText(err));
// 		return;
// 	}
	
// 	printf("Output devices:\n");
// #ifndef PA18API
// 	for (i = 0; i < Pa_GetDeviceCount(); ++i) {
// 		if (Pa_GetDeviceInfo(i)->maxOutputChannels) {
// 			printf("  %i - %s [%s]\n", i, Pa_GetDeviceInfo(i)->name, Pa_GetHostApiInfo(Pa_GetDeviceInfo(i)->hostApi)->name);
// 		}
// #else
// 	for (i = 0; i < Pa_CountDevices(); ++i) {
// 		printf("  %i - %s\n", i, Pa_GetDeviceInfo(i)->name);
// #endif
// 	}
// 	printf("\n");
	
// 	if ((err = Pa_Terminate()) != paNoError) {
// 		LOG_WARN("error closing port audio: %s", Pa_GetErrorText(err));
// 	}
}

void set_volume(unsigned left, unsigned right) {
	LOG_DEBUG("setting internal gain left: %u right: %u", left, right);
	// todo: determine if the DAC has special logic for GAIN value (e.g. is it a percentage, or a value from 0 to 254, etc.)
	LOCK;
	output.gainL = left;
	output.gainR = right;
	UNLOCK;
}

static int dac_device_id(const char *device) {
	int len = strlen(device);
	int i;

	// if (!strncmp(device, "default", 7)) {

	// 	return Pa_GetDefaultOutputDeviceID();

	// }
	// if (len >= 1 && len <= 2 && device[0] >= '0' && device[0] <= '9') {
	// 	return atoi(device);
	// }


	// for (i = 0; i < Pa_CountDevices(); ++i) {
	// 	if (!strncmp(Pa_GetDeviceInfo(i)->name, device, len)) {

	// 		return i;
	// 	}
	// }

	return -1;
}


// static int dac_callback(void *pa_input, void *pa_output, unsigned long pa_frames_wanted, 
// 			   PaTimestamp outTime, void *userData);

bool test_open(const char *device, unsigned rates[], bool userdef_rates) {
// 	PaStreamParameters outputParameters;
// 	PaError err;
// 	unsigned ref[] TEST_RATES;
// 	int device_id, i, ind;
// #if WIN
// 	PaWasapiStreamInfo wasapiInfo;
// 	const PaDeviceInfo * paDeviceInfo;
// 	const PaHostApiInfo *paHostApiInfo;

// #endif
// 	if ((device_id = dac_device_id(device)) == -1) {
// 		LOG_INFO("device %s not found", device);
// 		return false;
// 	}

// 	outputParameters.device = device_id;
// 	outputParameters.channelCount = 2;
// 	outputParameters.sampleFormat = paInt32;


// 	// check supported sample rates
// 	// Note use Pa_OpenStream as it appears more reliable than Pa_IsFormatSupported on some windows apis
// 	for (i = 0, ind = 0; ref[i]; ++i) {

// 		err = Pa_OpenStream(&pa.stream, paNoDevice, 0, 0, NULL, outputParameters.device,
// 			outputParameters.channelCount, outputParameters.sampleFormat, NULL, (double)ref[i],
// 			paFramesPerBuffer, paNumberOfBuffers, paNoFlag, dac_callback, NULL);

// 		switch (err) {
// 			case paInvalidSampleRate:
// 				continue;

// 			case paNoError:
// 				Pa_CloseStream(pa.stream);
// 				if (!userdef_rates) {
// 					rates[ind++] = ref[i];
// 				}
// 				continue;

// 			default:	
// 				/* Any other error is a failure */
// 				LOG_WARN("error opening portaudio stream: %s", Pa_GetErrorText(err));
// 				return false;
// 		}
// 	}

// 	if (!rates[0] && !userdef_rates) {
// 		LOG_WARN("no available rate found");
// 		return false;
// 	}

// 	pa.stream = NULL;
	return true;
}

static void pa_stream_finished(void *userdata) {
	// if (running) {
	// 	LOG_INFO("stream finished");
	// 	LOCK;
	// 	output.pa_reopen = true;
	// 	wake_controller();
	// 	UNLOCK;
	// }
}

static thread_type monitor_thread;
bool monitor_thread_running = false;

static void *dac_monitor() {
	// bool output_off;

	// LOCK;

	// if (monitor_thread_running) {
	// 	LOG_DEBUG("monitor thread already running");
	// 	UNLOCK;
	// 	return 0;
	// }

	// LOG_DEBUG("start monitor thread");

	// monitor_thread_running = true;
	// output_off = (output.state == OUTPUT_OFF);

	// while (monitor_thread_running) {
	// 	if (output_off) {
	// 		if (output.state != OUTPUT_OFF) {
	// 			LOG_INFO("output on");
	// 			break;
	// 		}
	// 	} else {
	// 		// this is a hack to partially support hot plugging of devices
	// 		// we rely on terminating and reinitalising PA to get an updated list of devices and use name for output.device
	// 		LOG_INFO("probing device %s", output.device);
	// 		Pa_Terminate();
	// 		Pa_Initialize();
	// 		pa.stream = NULL;
	// 		if (dac_device_id(output.device) != -1) {
	// 			LOG_INFO("device reopen");
	// 			break;
	// 		}
	// 	}

	// 	UNLOCK;
	// 	sleep(output_off ? 1 : 5);
	// 	LOCK;
	// }

	// LOG_DEBUG("end monitor thread");

	// monitor_thread_running = false;
	// pa.stream = NULL;
	//dac_open();

	UNLOCK;

	return 0;
}

void dac_open(void) {
	bool err=false;
// 	PaStreamParameters outputParameters;
// 	PaError err = paNoError;
// 	int device_id;
// #if WIN
// 	PaWasapiStreamInfo wasapiInfo;
// 	const PaDeviceInfo * paDeviceInfo;
// 	const PaHostApiInfo *paHostApiInfo;

// #endif
// 	if (pa.stream) {
// 		if ((err = Pa_CloseStream(pa.stream)) != paNoError) {
// 			LOG_WARN("error closing stream: %s", Pa_GetErrorText(err));
// 		}
// 	}

// 	if (output.state == OUTPUT_OFF) {
// 		// we get called when transitioning to OUTPUT_OFF to create the probe thread
// 		// set err to avoid opening device and logging messages
// 		err = 1;

// 	} else if ((device_id = dac_device_id(output.device)) == -1) {
// 		LOG_INFO("device %s not found", output.device);
// 		err = 1;

// 	} else {

// 		outputParameters.device = device_id;
// 		outputParameters.channelCount = 2;
// 		outputParameters.sampleFormat = paInt32;

// #if OSX && !defined(OSXPPC) || POSIX
// 		/* enable pro mode which aims to avoid resampling if possible */
// 		/* command line controls pa_hostapi_option which is -1 if not specified, 0 or 1 - choose playnice if -1 or 1 */
// 		PaMacCoreStreamInfo macInfo;
// 		unsigned long streamInfoFlags;
// 	 	if (output.pa_hostapi_option) {
// 			LOG_INFO("opening device in PlayNice mode");
// 			streamInfoFlags = paMacCorePlayNice;
// 		} else {
// 			LOG_INFO("opening device in Pro mode");
// 			streamInfoFlags = paMacCorePro;
// 		}
// 		PaMacCore_SetupStreamInfo(&macInfo, streamInfoFlags);
// 		outputParameters.hostApiSpecificStreamInfo = &macInfo;
// #endif

// 	}

// 	if (!err &&

// 		(err = Pa_OpenStream(&pa.stream, paNoDevice, 0, 0, NULL, outputParameters.device, outputParameters.channelCount,
// 							outputParameters.sampleFormat, NULL, (double)output.current_sample_rate, paFramesPerBuffer,
// 							paNumberOfBuffers, paDitherOff, dac_callback, NULL)) != paNoError) {
// 		LOG_WARN("error opening device %i - %s : %s", outputParameters.device, Pa_GetDeviceInfo(outputParameters.device)->name, 
// 				 Pa_GetErrorText(err));

// 	}

// 	if (!err) {

// 		LOG_INFO("opened device %i - %s at %u fpb %u nbf %u", outputParameters.device, Pa_GetDeviceInfo(outputParameters.device)->name,
// 				 (unsigned int)output.current_sample_rate, paFramesPerBuffer, paNumberOfBuffers);


// 		pa.rate = output.current_sample_rate;


// 		if ((err = Pa_StartStream(pa.stream)) != paNoError) {
// 			LOG_WARN("error starting stream: %s", Pa_GetErrorText(err));
// 		}


// 	}

// 	if (err && !monitor_thread_running) {
// 		vis_stop();

// 		// create a thread to check for output state change or device return
// #if LINUX || OSX || FREEBSD || POSIX 
// 		pthread_create(&monitor_thread, NULL, dac_monitor, NULL);
// #endif

// 	}

	output.error_opening = !err;
}

static u8_t *optr;

static int _write_frames(frames_t out_frames, bool silence, s32_t gainL, s32_t gainR,
						 s32_t cross_gain_in, s32_t cross_gain_out, s32_t **cross_ptr) {
	
	// if (!silence) {
		
	// 	if (output.fade == FADE_ACTIVE && output.fade_dir == FADE_CROSS && *cross_ptr) {
	// 		_apply_cross(outputbuf, out_frames, cross_gain_in, cross_gain_out, cross_ptr);
	// 	}
		
	// 	if (gainL != FIXED_ONE || gainR!= FIXED_ONE) {
	// 		_apply_gain(outputbuf, out_frames, gainL, gainR);
	// 	}

	// 	IF_DSD(
	// 		if (output.outfmt == DOP) {
	// 			update_dop((u32_t *) outputbuf->readp, out_frames, output.invert);
	// 		} else if (output.outfmt != PCM && output.invert)
	// 			dsd_invert((u32_t *) outputbuf->readp, out_frames);
	// 	)

	// 	memcpy(optr, outputbuf->readp, out_frames * BYTES_PER_FRAME);

	// } else {

	// 	u8_t *buf = silencebuf;

	// 	IF_DSD(
	// 		if (output.outfmt != DOP) {
	// 			buf = silencebuf_dsd;
	// 			update_dop((u32_t *) buf, out_frames, false); // don't invert silence
	// 		}
	// 	)

	// 	memcpy(optr, buf, out_frames * BYTES_PER_FRAME);
	// }
	
	// optr += out_frames * BYTES_PER_FRAME;

	return (int)out_frames;
}


static int dac_callback(){
	// todo: implement DAC Interrupt processing here.  
	// since ISR need to be processed as fast as possible, a flag 
	// that indicates ready state from the device would be appropriate

	
 int ret;
	// frames_t frames;

	// optr = (u8_t *)pa_output;

	// LOCK;


	// output.device_frames = 0;

	// output.updated = gettime_ms();
	// output.frames_played_dmp = output.frames_played;

	// do {
	// 	frames = _output_frames(pa_frames_wanted);
	// 	pa_frames_wanted -= frames;
	// } while (pa_frames_wanted > 0 && frames != 0);

	// if (pa_frames_wanted > 0) {
	// 	LOG_DEBUG("pad with silence");
	// 	memset(optr, 0, pa_frames_wanted * BYTES_PER_FRAME);
	// }

	// if (output.state == OUTPUT_OFF) {
	// 	LOG_INFO("output off");
	// 	ret = paComplete;
	// } else if (pa.rate != output.current_sample_rate) {
	// 	ret = paComplete;
	// } else {
	// 	ret = paContinue;
	// }

	// UNLOCK;

// #ifdef PA18API
// 	if ( ret == paComplete )
// 		pa_stream_finished (userData);
// #endif
	return ret;
}

void output_init_dsp(log_level level, const char *device, unsigned output_buf_size, char *params, unsigned rates[], unsigned rate_delay,
					unsigned idle) {
	bool err;

	unsigned pa_frames = 0;
	unsigned pa_nbufs = 0;


	// char *t = next_param(params, ':');
	// char *c = next_param(NULL, ':');
	// if (t) pa_frames  = atoi(t);
	// if (c) pa_nbufs = atoi(c);


	// loglevel = level;

	// LOG_INFO("init output");

	// memset(&output, 0, sizeof(output));


	// if ( pa_frames != 0 )
	// 	paFramesPerBuffer = pa_frames;
	// if ( pa_nbufs != 0 )
	// 	paNumberOfBuffers = pa_nbufs;

	// output.format = 0;
	// output.start_frames = 0;
	// output.write_cb = &_write_frames;
	// output.rate_delay = rate_delay;
	// pa.stream = NULL;


	// if ((err = Pa_Initialize()) != paNoError) {
	// 	LOG_WARN("error initialising port audio: %s", Pa_GetErrorText(err));
	// 	exit(0);
	// }

	output_init_common(level, device, output_buf_size, rates, idle);

	LOCK;

	dac_open();
	UNLOCK;
}

void output_close_dsp(void) {
	// PaError err;

	// LOG_INFO("close output");

	// LOCK;

	// running = false;
	// monitor_thread_running = false;

	// if (pa.stream) {
	// 	if ((err = Pa_AbortStream(pa.stream)) != paNoError) {
	// 		LOG_WARN("error closing stream: %s", Pa_GetErrorText(err));
	// 	}
	// }

	// if ((err = Pa_Terminate()) != paNoError) {
	// 	LOG_WARN("error closing port audio: %s", Pa_GetErrorText(err));
	// }

	// UNLOCK;

	output_close_common();
}

#endif // PORTAUDIO
