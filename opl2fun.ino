// https://docs.google.com/document/d/1l84JTpSqWExiOSZbDhcDflVmwDLP2fVsZtAa2XhSkeI/edit
// https://photos.google.com/photo/AF1QipOQX2p_RnjAfk_JceSfqYZ8eBOw6Eqzg_BN3ZBJ

#include <SPI.h>
#include <SoftwareSerial.h>
#include <MemoryFree.h>
#include <avr/pgmspace.h>

#define PIN_SHIFT 13
#define PIN_DATA  11
#define PIN_RESET 10
#define PIN_LATCH 9
#define PIN_ADDR  8
#define PIN_SERIAL2_RX 2
#define PIN_SERIAL2_TX 3

#define FLAT  0x10
#define FLAT2 0x20
#define SHARP 0x40
#define NONE  0xff

#define OP_1  0
#define OP_2  1
#define INST -1

#define MIDI_STATE_INIT      0
#define MIDI_STATE_EAT_N     1
#define MIDI_STATE_PARAM1    2
#define MIDI_STATE_PARAM2    3
#define MIDI_STATE_EAT_SYSEX 4

#define MIDI   Serial
#define DEBUG  Serial2
SoftwareSerial Serial2(PIN_SERIAL2_RX, PIN_SERIAL2_TX);
// SoftwareSerial is too slow for MIDI

#define OPL2_QUEUE_SIZE 256
byte opl2_sync;
byte opl2_reg[256];
int  opl2_queue[OPL2_QUEUE_SIZE];
int  opl2_queue_write_idx;
int  opl2_queue_read_idx;

byte insts_count;
byte insts[4];
byte inst;

byte time_play;
byte time_ppqn;

byte midi_state;
byte midi_byte;
byte midi_eat;
byte midi_status;
byte midi_param1;
byte midi_type;

byte scale_type;
byte scale_root;
byte scale_chord;
byte chord[4];

byte note2scale[128];

byte chord_cc_enabled;

#define NUM_SCALES 16
const PROGMEM byte scales[NUM_SCALES][12] = {
    // { scale_len, scale_degree_1, ..., scale_degree_n }
    { 7,   2, 4, 5, 7, 9, 11, 12,   14, 16, 17, 18 },         // 0   major
    { 7,   2, 3, 5, 7, 9, 10, 12,   14, 15, 17, 19 },         // 1   dorian
    { 7,   1, 3, 5, 7, 8, 10, 12,   13, 15, 17, 19 },         // 2   phrygian
    { 7,   2, 4, 6, 7, 9, 11, 12,   14, 16, 18, 19 },         // 3   lydian
    { 7,   2, 4, 5, 7, 9, 10, 12,   14, 16, 17, 19 },         // 4   mixolydian
    { 7,   2, 3, 5, 7, 8, 10, 12,   14, 15, 17, 19 },         // 5   minor
    { 7,   1, 3, 5, 6, 8, 10, 12,   13, 15, 17, 18 },         // 6   locrian
    { 7,   2, 3, 5, 7, 8, 11, 12,   14, 15, 17, 19 },         // 7   harmonic minor
    { 7,   2, 3, 5, 7, 9, 11, 12,   14, 15, 17, 19 },         // 8   melodic minor
    { 7,   2, 3, 5, 6, 8,  9, 11,   13, 14, 16, 17 },         // 9   diminished
    { 7,   1, 4, 6, 8, 10,11, 12,   13, 16, 17, 18 },         // 10  enigmatic
    { 6,      2, 4, 6, 8, 10, 12,   14, 16, 18, 20, 22 },     // 11  whole
    { 6,      3, 4, 7, 8, 11, 12,   15, 16, 19, 20, 23 },     // 12  augmented
    { 6,      3, 5, 6, 7, 10, 12,   15, 17, 18, 19, 21 },     // 13  blues
    { 5,          2, 4, 7, 9, 12,   14, 16, 19, 21, 24, 26 }, // 14  major pentatonic
    { 5,          3, 5, 7, 9, 12,   15, 17, 19, 21, 23, 26 }  // 15  minor pentatonic
};

#define NUM_CHORDS 34
const PROGMEM byte chords[NUM_CHORDS][3] = {
    // 0x1_ == flat, 0x2_ == flat2, 0x4_ == sharp
    { 2, 4, NONE },                  // 0   maj
    { 3, 4, NONE },                  // 1   sus4
    { 1, 4, NONE },                  // 2   sus2
    { 0x12, 0x14, NONE },            // 3   dim
    { 0x12, 0x14, 0x26 },            // 4   dim7
    { 2, 0x44, NONE },               // 5   aug
    { 2, 3, 4 },                     // 6   add4
    { 2, 4, 5 },                     // 7   6
    { 2, 4, 6 },                     // 8   7
    { 2, 4, 8 },                     // 9   add9
    { 2, 4, 10 },                    // 10  11
    { 2, 4, 0x4a },                  // 11  #11
    { 2, 4, 12 },                    // 12  13
    { 2, 0x14, NONE },               // 13  b5
    { 2, 4, 0x16 },                  // 14  b7
    { 2, 0x44, 0x16 },               // 15  7#5
    { 2, 0x14, 0x16 },               // 16  7b5

    { 0x12, 4, NONE },               // 17  min
    { 0x13, 4, NONE },               // 18  msus4
    { 0x11, 4, NONE },               // 19  msus2
    { 0x22, 0x14, NONE },            // 20  mdim
    { 0x22, 0x14, 0x26 },            // 21  mdim7
    { 0x12, 0x44, NONE },            // 22  maug
    { 0x12, 3, 4 },                  // 23  madd4
    { 0x12, 4, 5 },                  // 24  m6
    { 0x12, 4, 6 },                  // 25  m7
    { 0x12, 4, 8 },                  // 26  madd9
    { 0x12, 4, 10 },                 // 27  m11
    { 0x12, 4, 0x4a },               // 28  m#11
    { 0x12, 4, 12 },                 // 29  m13
    { 0x12, 0x44, NONE },            // 30  mb5
    { 0x12, 4, 0x16 },               // 31  mb7
    { 0x12, 0x44, 0x16 },            // 32  m7#5
    { 0x12, 0x14, 0x16 }             // 33  m7b5
};

const PROGMEM byte op2offset[2][9] = {
    { 0x00, 0x01, 0x02, 0x08, 0x09, 0x0a, 0x10, 0x11, 0x12 }, // OP_1
    { 0x03, 0x04, 0x05, 0x0b, 0x0c, 0x0d, 0x13, 0x14, 0x15 }  // OP_2
};

const PROGMEM byte note2freq[3][128] = {
    { // fnum lo
        0,  86, 91, 96, 102,108,115,121,129,136,145,153,162,172,182,193,
        205,217,230,243,2,  17, 34, 51, 69, 88, 109,131,154,178,204,231,
        4,  35, 68, 102,139,177,218,6,  52, 101,152,207,4,  35, 68, 102,
        139,177,218,6,  52, 101,152,207,4,  35, 68, 102,139,177,218,6,
        52, 101,152,207,4,  35, 68, 102,139,177,218,6,  52, 101,152,207,
        4,  35, 68, 102,139,177,218,6,  52, 101,152,207,4,  35, 68, 102,
        139,177,218,6,  52, 101,152,207,4,  35, 68, 102,139,177,218,6,
        52, 101,152,207,4,  35, 68, 102,139,177,218,6,  52, 101,152,207
    },
    { // fnum hi
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
        2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  2,  2,  2,  2,
        2,  2,  2,  3,  3,  3,  3,  3,  2,  2,  2,  2,  2,  2,  2,  3,
        3,  3,  3,  3,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,
        2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  2,  2,  2,  2,
        2,  2,  2,  3,  3,  3,  3,  3,  2,  2,  2,  2,  2,  2,  2,  3,
        3,  3,  3,  3,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3
    },
    { // block
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
        1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,
        2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,
        4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  5,  5,  5,  5,
        5,  5,  5,  5,  5,  5,  5,  5,  6,  6,  6,  6,  6,  6,  6,  6,
        6,  6,  6,  6,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7
    }
};

const PROGMEM byte cc2bit[7][128] = {
    { // 1 bit
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
        1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
        1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
        1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1
    },
    { // 2 bits
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
        1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
        2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
        2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
        3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,
        3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3
    },
    { // 3 bits
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
        2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
        3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,
        4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,
        5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,
        6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6,
        7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7
    },
    { // 4 bits
        0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,
        2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  3,
        4,  4,  4,  4,  4,  4,  4,  4,  5,  5,  5,  5,  5,  5,  5,  5,
        6,  6,  6,  6,  6,  6,  6,  6,  7,  7,  7,  7,  7,  7,  7,  7,
        8,  8,  8,  8,  8,  8,  8,  8,  9,  9,  9,  9,  9,  9,  9,  9,
        10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11,
        12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13,
        14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15
    },
    { // 5 bits
        0,  0,  0,  0,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  3,  3,
        4,  4,  4,  4,  5,  5,  5,  5,  6,  6,  6,  6,  7,  7,  7,  7,
        8,  8,  8,  8,  9,  9,  9,  9,  10, 10, 10, 10, 11, 11, 11, 11,
        12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15,
        16, 16, 16, 16, 17, 17, 17, 17, 18, 18, 18, 18, 19, 19, 19, 19,
        20, 20, 20, 20, 21, 21, 21, 21, 22, 22, 22, 22, 23, 23, 23, 23,
        24, 24, 24, 24, 25, 25, 25, 25, 26, 26, 26, 26, 27, 27, 27, 27,
        28, 28, 28, 28, 29, 29, 29, 29, 30, 30, 30, 30, 31, 31, 31, 31
    },
    { // 6 bits
        0,  0,  1,  1,  2,  2,  3,  3,  4,  4,  5,  5,  6,  6,  7,  7,
        8,  8,  9,  9,  10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15,
        16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23,
        24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31,
        32, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37, 38, 38, 39, 39,
        40, 40, 41, 41, 42, 42, 43, 43, 44, 44, 45, 45, 46, 46, 47, 47,
        48, 48, 49, 49, 50, 50, 51, 51, 52, 52, 53, 53, 54, 54, 55, 55,
        56, 56, 57, 57, 58, 58, 59, 59, 60, 60, 61, 61, 62, 62, 63, 63
    },
    { // 7 bits
        0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
        16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
        32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
        48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,
        64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
        80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95,
        96, 97, 98, 99, 100,101,102,103,104,105,106,107,108,109,110,111,
        112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127
    }
};

void setup() {
    DEBUG.begin(9600);
    MIDI.begin(31250);
    SPI.begin();

    pinMode(PIN_LATCH, OUTPUT);
    pinMode(PIN_ADDR,  OUTPUT);
    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_LATCH, HIGH);
    digitalWrite(PIN_RESET, HIGH);
    digitalWrite(PIN_ADDR,  LOW);

    time_play = 0;
    time_ppqn = 0;

    scale_type = 0;
    scale_root = 0;
    scale_chord = 0;
    fill_scale();

    midi_state = MIDI_STATE_INIT;

    opl2_queue_write_idx = 0;
    opl2_queue_read_idx = 0;

    chord_cc_enabled = 1;

    opl2_reset();
}

void loop() {
    while (MIDI.available() > 0) {
        midi_byte = MIDI.read();
        midi_type = midi_byte & 0xf0;
        switch (midi_state) {
            case MIDI_STATE_INIT:
                if (midi_type == 0xa0 || midi_type == 0xc0 || midi_type == 0xe0) {
                    // aftertouch, patch change, pitch bend (ignore)
                    midi_eat = 2;
                    midi_state = MIDI_STATE_EAT_N;
                } else if (midi_type == 0xd0) {
                    // channel pressure (ignore)
                    midi_eat = 1;
                    midi_state = MIDI_STATE_EAT_N;
                } else if (midi_type == 0x80 || midi_type == 0x90 || midi_type == 0xb0) {
                    // note on, note off, cc (accept)
                    midi_status = midi_byte;
                    midi_state = MIDI_STATE_PARAM1;
                } else if (midi_type == 0xf0) {
                    // sysex (ignore)
                    midi_state = MIDI_STATE_EAT_SYSEX;
                } else if (midi_type == 0xf0) {
                    // time (accept)
                    handle_midi_time(midi_byte);
                } else if (midi_byte <= 0x7f) {
                    // running status!
                    midi_param1 = midi_byte;
                    midi_state = MIDI_STATE_PARAM2;
                }
                break;
            case MIDI_STATE_PARAM1:
                midi_param1 = midi_byte;
                midi_state = MIDI_STATE_PARAM2;
                break;
            case MIDI_STATE_PARAM2:
                if (midi_status >= 0x90 && midi_status <= 0x9f) {
                    handle_midi_note(midi_status & 0x0f, midi_param1 & 0x7f, midi_byte & 0x7f);
                } else if (midi_status >= 0x80 && midi_status <= 0x8f) {
                    handle_midi_note(midi_status & 0x0f, midi_param1 & 0x7f, 0);
                } else if (midi_status >= 0xb0 && midi_status <= 0xbf) {
                    handle_midi_cc(midi_status & 0x0f, midi_param1 & 0x7f, midi_byte & 0x7f);
                }
                midi_state = MIDI_STATE_INIT;
                break;
            case MIDI_STATE_EAT_N:
                if (midi_eat > 0) {
                    midi_eat -= 1;
                }
                if (midi_eat == 0) {
                    midi_state = MIDI_STATE_INIT;
                }
                break;
            case MIDI_STATE_EAT_SYSEX:
                if (midi_byte == 0xf7) {
                    midi_state = MIDI_STATE_INIT;
                }
                break;
        }
    }
    if (opl2_queue_read_idx != opl2_queue_write_idx) {
        opl2_dequeue_one();
    }
}

void handle_midi_note(byte chan, byte note, byte vel) {
    byte fhi, flo, blk, i;

    if (chan < 6) {
        if (note == 0) return;
        // tonal
        if (chan > 1) {
            chan = 2;
            fill_chord(note);
        } else {
            chord[0] = note;
            chord[1] = NONE;
        }
        for (i = 0; i < 4; ++i) {
            note = chord[i];
            if (note == NONE) break;
            if (vel > 0) {
                flo = pgm_read_byte(&note2freq[0][note & 0x7f]);
                fhi = pgm_read_byte(&note2freq[1][note & 0x7f]);
                blk = pgm_read_byte(&note2freq[2][note & 0x7f]);
                opl2_write_bits_scaled(0x40, chan + i, OP_2, 0, 6, 127-vel);
                // opl2_write_bits_scaled(0x40, chan + i, OP_1, 0, 6, 127-vel);
                opl2_write(0xa0 + chan + i, flo);
                opl2_write(0xb0 + chan + i, 0x20 | (blk << 2) | fhi);
            } else {
                opl2_write_bits(0xb0, chan + i, INST, 5, 1, 0);
            }
        }
    } else if (chan < 11 && note > 0) {
        // percussion
        opl2_write_bits(0xbd, 0, INST, 4 - (chan - 6), 1, vel > 0 ? 1 : 0);
    }
}

void handle_midi_cc(byte ichan, byte cc, byte val) {
    byte chan, start_chan, end_chan, tmp;

    // handle channels 0 thru 8 (9 instruments)
    if (ichan > 8) return;

    // if chord_cc_enabled, apply cc to all chord instruments
    if (chord_cc_enabled && ichan > 1) {
        start_chan = 2;
        end_chan = 5;
    } else {
        start_chan = ichan;
        end_chan = ichan;
    }

    for (chan = start_chan; chan <= end_chan; ++chan) {
        switch (cc) {
            case  2: tmp = pgm_read_byte(&cc2bit[3][val]);
                     opl2_write_bits_scaled(0xa0, chan, INST, 5, 2, tmp >> 2);       // freq_hi4
                     opl2_write_bits_scaled(0xb0, chan, INST, 0, 2, tmp & 0x03);
                     break;
            case  3: opl2_write_bits_scaled(0xa0, chan, INST, 0, 6, val);     break; // freq_lo6
            case  4: opl2_write_bits_scaled(0xb0, chan, INST, 2, 3, val);     break; // freq_blk

            case  5: opl2_write_bits_scaled(0xc0, chan, INST, 1, 3, val);     break; // inst_feedback

            case  6: opl2_write_bits_scaled(0xe0, chan, OP_2, 0, 2, val);     break; // op2_waveform
            case  7: opl2_write_bits_scaled(0xe0, chan, OP_1, 0, 2, val);     break; // op1_waveform

            case  8: opl2_write_bits_scaled(0x40, chan, OP_2, 0, 6, 127-val); break; // op2_level
            case  9: opl2_write_bits_scaled(0x40, chan, OP_1, 0, 6, 127-val); break; // op1_level

            case 10: opl2_write_bits_scaled(0x20, chan, OP_2, 0, 4, val);     break; // op2_modfreq
            case 11: opl2_write_bits_scaled(0x20, chan, OP_1, 0, 4, val);     break; // op1_modfreq

            case 12: opl2_write_bits_scaled(0x60, chan, OP_2, 4, 4, val);     break; // op2_env_attack
            case 13: opl2_write_bits_scaled(0x60, chan, OP_2, 0, 4, val);     break; // op2_env_decay
            case 14: opl2_write_bits_scaled(0x80, chan, OP_2, 4, 4, 127-val); break; // op2_env_sustain
            case 15: opl2_write_bits_scaled(0x80, chan, OP_2, 0, 4, 127-val); break; // op2_env_release

            case 16: opl2_write_bits_scaled(0x60, chan, OP_1, 4, 4, val);     break; // op1_env_attack
            case 17: opl2_write_bits_scaled(0x60, chan, OP_1, 0, 4, val);     break; // op1_env_decay
            case 18: opl2_write_bits_scaled(0x80, chan, OP_1, 4, 4, 127-val); break; // op1_env_sustain
            case 19: opl2_write_bits_scaled(0x80, chan, OP_1, 0, 4, 127-val); break; // op1_env_release

            case 20: opl2_write_bits_scaled(0x40, chan, OP_2, 6, 2, val);     break; // op2_levscale
            case 21: opl2_write_bits_scaled(0x40, chan, OP_1, 6, 2, val);     break; // op1_levscale

            case 22: opl2_write_bits_scaled(0x20, chan, OP_2, 7, 1, val);     break; // op2_ampmod
            case 23: opl2_write_bits_scaled(0x20, chan, OP_2, 6, 1, val);     break; // op2_vibrato
            case 24: opl2_write_bits_scaled(0x20, chan, OP_2, 5, 1, val);     break; // op2_sustain_on
            case 25: opl2_write_bits_scaled(0x20, chan, OP_2, 4, 1, val);     break; // op2_envscale

            case 26: opl2_write_bits_scaled(0x20, chan, OP_1, 7, 1, val);     break; // op1_ampmod
            case 27: opl2_write_bits_scaled(0x20, chan, OP_1, 6, 1, val);     break; // op1_vibrato
            case 28: opl2_write_bits_scaled(0x20, chan, OP_1, 5, 1, val);     break; // op1_sustain_on
            case 29: opl2_write_bits_scaled(0x20, chan, OP_1, 4, 1, val);     break; // op1_envscale

            case 30: opl2_write_bits_scaled(0xc0, chan, INST, 0, 1, val);     break; // inst_fm

            case 31: scale_root  = val % 12;         fill_scale(); goto cc_done;     // scale_root
            case 32: scale_type  = val % NUM_SCALES; fill_scale(); goto cc_done;     // scale_type
            case 33: scale_chord = val % NUM_CHORDS; goto cc_done;                   // scale_chord

            // TODO lfo

            // TODO trigcond

            // TODO trigfx

            // TODO arp

            // TODO global
            //      amdep, vibdep, chord_cc_enabled
        }
    }
cc_done: (void)cc;

}

void handle_midi_time(byte val) {
    switch (val) {
        case 0xfa: // start
            time_ppqn = 0;
            time_play = 1;
            break;
        case 0xfb: // continue
            time_play = 1;
            break;
        case 0xfc: // stop
            time_play = 0;
            break;
        case 0xf8: // clock
            if (time_play == 1) {
                time_ppqn += 1;
                if (time_ppqn > 23) {
                    time_ppqn = 0;
                }
            }
            break;
    }
}

void fill_chord(byte note) {
    byte degree_acc, i;
    int tmp;

    note = note2scale[note];
    chord[0] = note;

    for (i = 0; i < 3; i++) {
        degree_acc = pgm_read_byte(&chords[scale_chord][i]);
        if (degree_acc == NONE) {
            chord[i+1] = NONE;
            break;
        }
        tmp = note + pgm_read_byte(&scales[scale_type][degree_acc & 0x0f]);
        if (degree_acc & FLAT) {
            tmp -= 1;
        } else if (degree_acc & FLAT2) {
            tmp -= 2;
        } else if (degree_acc & SHARP) {
            tmp += 1;
        }
        while (tmp < 0)   tmp += 12;
        while (tmp > 127) tmp -= 12;
        chord[i+1] = (byte)tmp;
    }
}

void fill_scale() {
    byte note, i, j, last_deg, deg_i, scale_len;
    note = scale_root;
    i = 0;
    j = 0;
    last_deg = 0;
    deg_i = 1;
    scale_len = pgm_read_byte(&scales[scale_type][0]);
    while (1) {
        for (i = last_deg; i < pgm_read_byte(&scales[scale_type][deg_i]); ++i) {
            note2scale[j] = note;
            j += 1;
            if (j == 127) return;
        }
        last_deg = pgm_read_byte(&scales[scale_type][deg_i]);
        deg_i += 1;
        if (deg_i > scale_len) {
            last_deg = 0;
            deg_i = 1;
        }
        note += pgm_read_byte(&scales[scale_type][deg_i]) - last_deg;
    }
}

void opl2_write_bits_scaled(byte sreg, byte inst, int op, byte bitoffset, byte nbits, byte data) {
    byte val;
    val = pgm_read_byte(&cc2bit[nbits-1][data]);
    opl2_write_bits(sreg, inst, op, bitoffset, nbits, val);
}

void opl2_write_bits(byte sreg, byte inst, int op, byte bitoffset, byte nbits, byte val) {
    byte reg, mask;
    reg = sreg + (op == INST ? inst : pgm_read_byte(&op2offset[op][inst]));
    mask = ~(((1 << nbits) - 1) << bitoffset);
    val = val << bitoffset;
    opl2_write(reg, (opl2_reg[reg] & mask) | val);
}

void opl2_reset() {
    int i;

    // enable sync writes
    opl2_sync = 1;

    // reset
    digitalWrite(PIN_RESET, LOW);
    delay(1);
    digitalWrite(PIN_RESET, HIGH);

    // zero out all registers
    memset(opl2_reg, 0xff, 256);
    for (i = 0; i <= 0xff; ++i) opl2_write(i, 0x00);

    // enable waveform select
    opl2_write(0x01, 0x20);

    // enable rhythm mode
    opl2_write(0xbd, 0x20);

    // init operators for perc/tonal
    for (i = 0; i < 9; ++i) {
        opl2_write_bits(0x60, i, OP_1, 0, 4, i >= 6 ? 0x08 : 0x02); // decay=short/long
        opl2_write_bits(0x60, i, OP_2, 0, 4, i >= 6 ? 0x08 : 0x02);

        opl2_write_bits(0x60, i, OP_1, 4, 4, 0x0f); // attack=fast
        opl2_write_bits(0x60, i, OP_2, 4, 4, 0x0f);

        opl2_write_bits(0x80, i, OP_1, 0, 4, i >= 6 ? 0x08 : 0x04); // release=short/long
        opl2_write_bits(0x80, i, OP_2, 0, 4, i >= 6 ? 0x08 : 0x04);

        opl2_write_bits(0x80, i, OP_1, 4, 4, i >= 6 ? 0x0f : 0x07); // sustain=high/medium
        opl2_write_bits(0x80, i, OP_2, 4, 4, i >= 6 ? 0x0f : 0x07);

        opl2_write_bits(0x20, i, OP_1, 0, 4, 0x01); // freq multiple=1
        opl2_write_bits(0x20, i, OP_2, 0, 4, 0x01);

        opl2_write_bits(0x20, i, OP_1, 5, 1, i >= 6 ? 0x00 : 0x01); // env release=off/on
        opl2_write_bits(0x20, i, OP_2, 5, 1, i >= 6 ? 0x00 : 0x01);

        opl2_write(0xa0 + i, 0x8b); // medium freq
        opl2_write(0xb0 + i, 0x06);
    }

    // disable sync writes
    opl2_sync = 1;
}

void opl2_write_real(byte reg, byte data) {
    // exit early if no change
    if (opl2_reg[reg] == data) return;

    // send address
    digitalWrite(PIN_ADDR, LOW);
    SPI.transfer(reg);
    digitalWrite(PIN_LATCH, LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_LATCH, HIGH);
    delayMicroseconds(4);

    // send value
    digitalWrite(PIN_ADDR, HIGH);
    SPI.transfer(data);
    digitalWrite(PIN_LATCH, LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_LATCH, HIGH);
    delayMicroseconds(23);

    // remember value
    opl2_reg[reg] = data;
}

void opl2_write(byte reg, byte data) {
    if (opl2_sync) {
        opl2_write_real(reg, data);
    } else {
        opl2_queue[opl2_queue_write_idx] = ((int)reg << 8) | (int)data;
        opl2_queue_write_idx += 1;
        if (opl2_queue_write_idx >= OPL2_QUEUE_SIZE) opl2_queue_write_idx = 0;
    }
}

void opl2_dequeue_one() {
    int reg_data;
    reg_data = opl2_queue[opl2_queue_read_idx];
    opl2_write_real((reg_data & 0xff00) >> 8, reg_data & 0xff);
    opl2_queue_read_idx += 1;
    if (opl2_queue_read_idx >= OPL2_QUEUE_SIZE) opl2_queue_read_idx = 0;
}

/*
void beep() {
    opl2_write(0x20, 0x01); // 20    01    Set the modulator's multiple to 1
    opl2_write(0x40, 0x10); // 40    10    Set the modulator's level to about 40 dB
    opl2_write(0x60, 0xf0); // 60    F0    Modulator attack: quick; decay: long
    opl2_write(0x80, 0x77); // 80    77    Modulator sustain: medium; release: medium
    opl2_write(0xa0, 0x98); // A0    98    Set voice frequency's LSB (it'll be a D#)

    opl2_write(0x23, 0x01); // 23    01    Set the carrier's multiple to 1
    opl2_write(0x43, 0x00); // 43    00    Set the carrier to maximum volume (about 47 dB)
    opl2_write(0x63, 0xf0); // 63    F0    Carrier attack: quick; decay: long
    opl2_write(0x83, 0x77); // 83    77    Carrier sustain: medium; release: medium
    opl2_write(0xb0, 0x31); // B0    31    Turn the voice on; set the octave and freq MSB
    delay(1000);

    opl2_write(0xb0, 0x11);
    delay(1000);
}


byte t=36;
void loop() {
  scale_type=2;
  handle_midi_cc(2, 33, t);
    handle_midi_note(2, t, 0xff);
    delay(1000);
    handle_midi_note(2, t, 0x00);
    delay(50);
    t += 1;
    if (t == 100) t=36;

Debug("freeMemory()=");
Debug(freeMemory());

}
*/
