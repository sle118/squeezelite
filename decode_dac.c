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

// Takes a byte stream from the input buffer and pushes it 
// to the output buffer;  decoding is performed on the DAC
// itself.

#include "squeezelite.h"

extern log_level loglevel;

extern struct buffer *streambuf;
extern struct buffer *outputbuf;
extern struct streamstate stream;
extern struct outputstate output;
extern struct decodestate decode;
extern struct processstate process;


#define LOCK_S   mutex_lock(streambuf->mutex)
#define UNLOCK_S mutex_unlock(streambuf->mutex)
#define LOCK_O   mutex_lock(outputbuf->mutex)
#define UNLOCK_O mutex_unlock(outputbuf->mutex)
#if PROCESS
#define LOCK_O_direct   if (decode.direct) mutex_lock(outputbuf->mutex)
#define UNLOCK_O_direct if (decode.direct) mutex_unlock(outputbuf->mutex)
#define LOCK_O_not_direct   if (!decode.direct) mutex_lock(outputbuf->mutex)
#define UNLOCK_O_not_direct if (!decode.direct) mutex_unlock(outputbuf->mutex)
#define IF_DIRECT(x)    if (decode.direct) { x }
#define IF_PROCESS(x)   if (!decode.direct) { x }
#else
#define LOCK_O_direct   mutex_lock(outputbuf->mutex)
#define UNLOCK_O_direct mutex_unlock(outputbuf->mutex)
#define LOCK_O_not_direct
#define UNLOCK_O_not_direct
#define IF_DIRECT(x)    { x }
#define IF_PROCESS(x)
#endif

#define MAX_DECODE_FRAMES 4096

static u32_t sample_rate;
static u32_t sample_size;
static u32_t channels;
static bool  bigendian;
static bool  limit;
static u32_t audio_left;
static u32_t bytes_per_frame;

static decode_state dac_decode(void) {
	unsigned bytes, in, out;
	frames_t frames, count;
	u8_t *optr;
	u8_t *iptr;
	u8_t tmp[16];
	
	LOCK_S;

	LOCK_O_direct;

	bytes = min(_buf_used(streambuf), _buf_cont_read(streambuf));

	IF_DIRECT(
		out = min(_buf_space(outputbuf), _buf_cont_write(outputbuf)) / BYTES_PER_FRAME;
	);
	IF_PROCESS(
		out = process.max_in_frames;
	);

	if ((stream.state <= DISCONNECT && bytes == 0) || (limit && audio_left == 0)) {
		UNLOCK_O_direct;
		UNLOCK_S;
		return DECODE_COMPLETE;
	}

	if (decode.new_stream) {
		LOG_INFO("setting track_start");
		LOCK_O_not_direct;
		output.track_start = outputbuf->writep;
		decode.new_stream = false;

		output.next_sample_rate = decode_newstream(sample_rate, output.supported_rates);
		if (output.fade_mode) _checkfade(true);

		UNLOCK_O_not_direct;
		IF_PROCESS(
			out = process.max_in_frames;
		);
		bytes_per_frame = 1;
	}

	IF_DIRECT(
		optr = (u8_t *)outputbuf->writep;
	);
	IF_PROCESS(
		optr = (u8_t *)process.inbuf;
	);
	iptr = (u8_t *)streambuf->readp;

	in = bytes / bytes_per_frame;

	//  handle frame wrapping round end of streambuf
	//  - only need if resizing of streambuf does not avoid this, could occur in localfile case
	if (in == 0 && bytes > 0 && _buf_used(streambuf) >= bytes_per_frame) {
		memcpy(tmp, iptr, bytes);
		memcpy(tmp + bytes, streambuf->buf, bytes_per_frame - bytes);
		iptr = tmp;
		in = 1;
	}

	frames = min(in, out);
	frames = min(frames, MAX_DECODE_FRAMES);

	if (limit && frames * bytes_per_frame > audio_left) {
		LOG_INFO("reached end of audio");
		frames = audio_left / bytes_per_frame;
	}

	count = frames * BYTES_PER_FRAME ; // we've established the number of frames to process, not convert to bytes

	while (count--) {
		*optr++ = *iptr++;
	}
	


	LOG_SDEBUG("decoded %u frames", frames);

	_buf_inc_readp(streambuf, frames * bytes_per_frame);

	if (limit) {
		audio_left -= frames * bytes_per_frame;
	}

	IF_DIRECT(
		_buf_inc_writep(outputbuf, frames * BYTES_PER_FRAME);
	);
	IF_PROCESS(
		process.in_frames = frames;
	);

	UNLOCK_O_direct;
	UNLOCK_S;

	return DECODE_RUNNING;
}

static void dac_open_stream(u8_t size, u8_t rate, u8_t chan, u8_t endianness) {
	// sample_size = size - '0' + 1;
	// sample_rate = sample_rates[rate - '0'];
	// channels    = chan - '0';
	// bigendian   = (endianness == '0');
	// limit       = false;
	LOG_INFO("vs1053 size: %u rate: %u chan: %u bigendian: %u", sample_size, sample_rate, channels, bigendian);
	//buf_adjust(streambuf, sample_size * channels);
}

static void dac_close(void) {
	//buf_adjust(streambuf, 1);
}
#define READ_SIZE 1024 // limited resources on the platform, and we should push frames as fast as possible to the DAC
#define WRITE_SIZE 2048 // no need for a lot of room;  we are pushing bytes 1:1 and the dac should be able to sustain the throughput
struct codec *register_dac(const char *codec) {

	if (!strcmp(codec, "pcm")) {
		static struct codec ret = { 
			'p',         // id
			"aif,pcm", // types
			READ_SIZE,        // min read
			WRITE_SIZE,      // min space
			dac_open_stream,    // open
			dac_close,   // close
			dac_decode,  // decode
		};
	
	LOG_INFO("using dac to decode aif,pcm");
	return &ret;
	}
	if (!strcmp(codec, "wma")) {

		static struct codec ret = { 
			'w',         // id
			"wma,wmap,wmal", // types
			READ_SIZE,        // min read
			WRITE_SIZE,      // min space
			dac_open_stream,    // open
			dac_close,   // close
			dac_decode,  // decode
		};
		
		LOG_INFO("using dac to decode wma,wmap,wmal");
		return &ret;
	}

	if (!strcmp(codec, "alc")) {

		static struct codec ret = { 
			'l',         // id
			"alc",       // types
			READ_SIZE,        // min read
			WRITE_SIZE,      // min space
			dac_open_stream,    // open
			dac_close,   // close
			dac_decode,  // decode
		};
		
		LOG_INFO("using ffmpeg to decode alc");		
		return &ret;
	}
	if (!strcmp(codec, "aac")) {

		static struct codec ret = { 
		'a',          // id
		"aac",        // types
			READ_SIZE,        // min read
			WRITE_SIZE,      // min space
			dac_open_stream,    // open
			dac_close,   // close
			dac_decode,  // decode
		};
		
		LOG_INFO("using dac to decode alc");		
		return &ret;
	}
	if (!strcmp(codec, "ogg")) {

		static struct codec ret = { 
		'o',          // id
		"ogg",        // types
			READ_SIZE,        // min read
			WRITE_SIZE,      // min space
			dac_open_stream,    // open
			dac_close,   // close
			dac_decode,  // decode
		};
		
		
		LOG_INFO("using dac to decode ogg");		
		return &ret;
	}
		if (!strcmp(codec, "flac")) {

		static struct codec ret = { 
		'f',          // id
		"flc",        // types
			READ_SIZE,        // min read
			WRITE_SIZE,      // min space
			dac_open_stream,    // open
			dac_close,   // close
			dac_decode,  // decode
		};
		
		
		LOG_INFO("using dac to decode flac");		
		return &ret;
	}
		if (!strcmp(codec, "mp3")) {

		static struct codec ret = { 
		'm',          // id
		"mp3",        // types
			READ_SIZE,        // min read
			WRITE_SIZE,      // min space
			dac_open_stream,    // open
			dac_close,   // close
			dac_decode,  // decode
		};
		
		
		LOG_INFO("using dac to decode ZZZZ");		
		return &ret;
	}
		
	


	return NULL;
}
