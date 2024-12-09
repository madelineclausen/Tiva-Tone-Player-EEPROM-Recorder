#include <stdint.h>
#include "tm4c123gh6pm.h"

#define PLAY     0
#define RECORD   1
#define PLAYBACK 2

// Constants
#define MAX_NOTES 100
#define EEPROM_ADDRESS 0x50

#define ROW_MASK                 (0x3C)    // PA2-PA5 (Rows)
#define COLUMN_MASK_A            (0xC0)    // PA6, PA7 (Columns on Port A)
#define COLUMN_MASK_C            (0xC0)    // PC6, PC7 (Columns on Port C)

// Keypad configuration
#define ROWS 4
#define COLS 4
char keypad[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

// Global variables
volatile int mode = PLAY;           // Current mode
volatile int noteCount = 0;         // Number of recorded notes
uint8_t sequence[MAX_NOTES];        // In-memory storage of sequence
int currentIndex = 0;               // Playback index

// Function prototypes
void initSystem();
void initKeypad();
void initPWM();
void initEEPROM();
void initLEDs();
void playTone(int frequency);
void stopTone();
void switchMode(char key);
void recordKey(char key);
void playbackSequence();
void setLEDs(int mode);
int keyToFrequency(char key);
void delayMs(int ms);

// EEPROM functions
void writeEEPROM(uint8_t address, char* data, uint8_t bytes_count);
char* readEEPROM(uint8_t address, uint8_t bytes_count);
void clearEEPROM();
static int I2C_wait_till_done(void);

// Keypad functions
char getKey();

int main() {
    initSystem();

    while (1) {
        char key = getKey();

        if (key != 0) {
            if (key >= '1' && key <= '8' && mode != PLAYBACK) {
                playTone(keyToFrequency(key)); // Play the corresponding frequency
                if (mode == RECORD) {
                    recordKey(key);
                    delayMs(1);
                }
            } else if (key == 'A' || key == 'B' || key == 'C') {
                switchMode(key); // Handle mode switching
            } else if (key == 'D' && mode == PLAYBACK) {
                playbackSequence(); // Handle playback
            }
        } else {
            stopTone(); // Stop the tone when no key is pressed
        }
    }
}

// System initialization
void initSystem() {
    initKeypad();
    initPWM();
    initEEPROM();
    initLEDs();

    setLEDs(PLAY); // Default mode is Play
}

// Initialize keypad
void initKeypad() {
    // Enable clocks for Port A (Rows) and Port C (Columns)
    SYSCTL_RCGCGPIO_R |= (1U << 0) | (1U << 2);
    delayMs(40);

    // Set up keypad rows (PA2-PA5) as outputs
    GPIO_PORTA_DIR_R |= ROW_MASK;
    GPIO_PORTA_PUR_R |= ROW_MASK;
    GPIO_PORTA_DEN_R |= ROW_MASK;

    // Set up keypad columns (PA6, PA7, PC6, PC7) as inputs
    GPIO_PORTA_DIR_R &= ~COLUMN_MASK_A;
    GPIO_PORTA_PUR_R |= COLUMN_MASK_A;
    GPIO_PORTA_DEN_R |= COLUMN_MASK_A;

    GPIO_PORTC_DIR_R &= ~COLUMN_MASK_C;
    GPIO_PORTC_PUR_R |= COLUMN_MASK_C;
    GPIO_PORTC_DEN_R |= COLUMN_MASK_C;
}

// Initialize PWM
void initPWM() {
    SYSCTL_RCGCPWM_R |= 2;       /* Enable clock to PWM1 module */
    SYSCTL_RCGCGPIO_R |= 0x20;   /* Enable system clock to PORTF */
    SYSCTL_RCC_R &= ~0x00100000; /* Directly feed clock to PWM1 module without pre-divider */

    /* Setting of PF2 pin for M1PWM6 channel output pin */
    GPIO_PORTF_AFSEL_R |= (1<<2);     /* PF2 sets a alternate function */
    GPIO_PORTF_PCTL_R &= ~0x00000F00; /*set PF2 as output pin */
    GPIO_PORTF_PCTL_R |= 0x00000500; /* make PF2 PWM output pin */
    GPIO_PORTF_DEN_R |= (1<<2);      /* set PF2 as a digital pin */

    /* PWM1 channel 6 setting */
    PWM1_3_CTL_R &= ~(1<<0);   /* Disable Generator 3 counter */
    PWM1_3_CTL_R &= ~(1<<1);   /* select down count mode of counter 3*/
    PWM1_3_GENA_R = 0x0000008C; /* Set PWM output when counter reloaded and clear when matches PWMCMPA */
    PWM1_3_LOAD_R = 16000;   /* set load value for 1kHz (16MHz/16000) */
    PWM1_3_CMPA_R = 8000 - 1;  /* set duty cyle to 50% by loading of 16000 to PWM1CMPA */
    PWM1_3_CTL_R = 1;         /* Enable Generator 3 counter */
    PWM1_ENABLE_R = 0x40;      /* Enable PWM1 channel 6 output */
}

// Initialize EEPROM
void initEEPROM() {
    SYSCTL_RCGCI2C_R |= 0x00000008; // Enable the clock for I2C 3
    volatile int delay1 = SYSCTL_RCGCI2C_R; // Dummy read for I2C stabilization
    SYSCTL_RCGCGPIO_R |= (1U << 3); // Enable the clock for port D
    volatile int delay2 = SYSCTL_RCGCGPIO_R; // Dummy read for stabilization
    GPIO_PORTD_DEN_R |= (1U << 0) | (1U << 1); // Assert DEN for port D
    // Configure Port D pins 0 and 1 as I2C 3
    GPIO_PORTD_AFSEL_R |= 0x00000003;
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & ~0x000000FF) | 0x00000033; // PD0=SCL, PD1=SDA
    GPIO_PORTD_ODR_R |= 0x00000002; // SDA (PD1 ) pin as open darin
    I2C3_MCR_R = 0x0010; // Enable I2C 3 master function
    I2C3_MTPR_R = 7;

//    SYSCTL_RCGCGPIO_R |= 0x08; // Enable clock for Port D
//    SYSCTL_RCGCI2C_R |= 0x08;  // Enable clock for I2C3
//
//    // Configure PD6 (SCL) and PD7 (SDA)
//    GPIO_PORTD_DEN_R |= (1 << 6) | (1 << 7);
//    GPIO_PORTD_AFSEL_R |= (1 << 6) | (1 << 7);
//    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R & 0x00FFFFFF) | 0x33000000; // I2C3 on PD6/PD7
//    GPIO_PORTD_ODR_R |= (1 << 7); // Open-drain for SDA (PD7)
//    GPIO_PORTD_PUR_R |= (1 << 6) | (1 << 7); // Enable pull-ups
//
//    // Configure I2C3
//    I2C3_MCR_R = 0x10; // Enable master mode
//    I2C3_MTPR_R = 7;   // Set SCL clock speed (100 kHz for 16 MHz system clock)
}

// Initialize LEDs
void initLEDs() {
    // Enable Port B for LEDs
    SYSCTL_RCGCGPIO_R |= 0x02; // Enable GPIO Port B
    while ((SYSCTL_PRGPIO_R & 0x02) == 0); // Wait for Port B ready

    GPIO_PORTB_DIR_R |= 0x07; // Set PB0-PB2 as outputs
    GPIO_PORTB_DEN_R |= 0x07; // Enable digital functionality on PB0-PB2
}

// Play a tone using PWM
void playTone(int frequency) {
    if (frequency > 0) {
        int load = 16000000 / frequency - 1; // Calculate the load value
        PWM1_3_LOAD_R = load;               // Set the load value
        PWM1_3_CMPA_R = load / 2;           // Set duty cycle to 50%
        PWM1_ENABLE_R = 0x40;               // Enable PWM output on PF2
    }
}

// Stop the tone
void stopTone() {
    PWM1_ENABLE_R &= ~0x40; // Disable PWM output on PF2
}

// Switch between modes
void switchMode(char key) {
    if (key == 'A') {
        mode = PLAY;
        setLEDs(PLAY);
    } else if (key == 'B') {
        mode = RECORD;
        setLEDs(RECORD);
        noteCount = 0; // Clear previous recordings
        //clearEEPROM(); // Clear EEPROM recordings
    } else if (key == 'C') {
        mode = PLAYBACK;
        setLEDs(PLAYBACK);
    }
}

// Record a key press
void recordKey(char key) {
    if (noteCount < MAX_NOTES) {
        sequence[noteCount++] = key;
        char data[1];
        data[0] = key;
        writeEEPROM(noteCount - 1, data, 1); // Save to EEPROM                  // ERROR HERE: WRITE ISN'T WRITING
        //char* test = readEEPROM(noteCount - 1, 1);
        delayMs(1);
    }
}

// Playback the recorded sequence
void playbackSequence() {
    int i;
    for (i = 0; i < noteCount; i++) {
        char* key = readEEPROM(i, 1); // sequence[i]; // Read from EEPROM        // TEST WITH ARRAY HERE
        playTone(keyToFrequency(*key));
        delayMs(1000); // Play each note for 1 second
        stopTone();
        delayMs(100); // Small gap between notes
    }
}

// Set LEDs based on mode
void setLEDs(int mode) {
    GPIO_PORTB_DATA_R &= ~0x07; // Clear PB0-PB2
    GPIO_PORTB_DATA_R |= (1 << mode); // Light up the corresponding LED
}

// Map keypad key to frequency
int keyToFrequency(char key) {
    switch (key) {
        case '1': return 262; // C
        case '2': return 294; // D
        case '3': return 330; // E
        case '4': return 349; // F
        case '5': return 392; // G
        case '6': return 440; // A
        case '7': return 494; // B
        case '8': return 523; // C (octave)
        default: return 0;
    }
}

// Delay in milliseconds
void delayMs(int ms) {
    int i;
    for (i = 0; i < ms*3180; i++) {
    }
}

static int I2C_wait_till_done(void)
{
    while(I2C3_MCS_R & 1);   /* wait until I2C master is not busy */
    return I2C3_MCS_R & 0xE; /* return I2C error code, 0 if no error*/
}

// Write Data to EEPROM
void writeEEPROM(uint8_t address, char* data, uint8_t bytes_count) {
    if (bytes_count == 0) return; // No data to write

    while (I2C3_MCS_R & 0x01); // Wait for I2C bus to be idle

    I2C3_MSA_R = EEPROM_ADDRESS << 1; // Set EEPROM address (R/W = 0)
    I2C3_MDR_R = (address >> 8) & 0xFF; // Send high byte of address
    I2C3_MCS_R = 0x03; // START + RUN
    if (I2C_wait_till_done()) return; // Error handling

    I2C3_MDR_R = address & 0xFF; // Send low byte of address
    I2C3_MCS_R = 0x01; // RUN
    if (I2C_wait_till_done()) return;

    // Send data bytes
    int i;
    for (i = 0; i < bytes_count; i++) {
        I2C3_MDR_R = data[i];
        I2C3_MCS_R = (i == bytes_count - 1) ? 0x05 : 0x01; // STOP for last byte
        if (I2C_wait_till_done()) return;
    }

    while (I2C3_MCS_R & 0x40); // Wait for bus to be idle
}

//void writeEEPROM(uint8_t address, char* data, uint8_t bytes_count) {
//    char error;
//    if (bytes_count <= 0)
//        return -1;                  /* no write was performed */
//    /* send slave address and starting address */
//    I2C3_MSA_R = EEPROM_ADDRESS << 1;
//    I2C3_MDR_R = address;
//    I2C3_MCS_R = 3;                  /* S-(saddr+w)-ACK-maddr-ACK */
//
//    error = I2C_wait_till_done();   /* wait until write is complete */
//    if (error) return error;
//
//    /* send data one byte at a time */
//    while (bytes_count > 1)
//    {
//        I2C3_MDR_R = *data++;             /* write the next byte */
//        I2C3_MCS_R = 1;                   /* -data-ACK- */
//        error = I2C_wait_till_done();
//        if (error) return error;
//        bytes_count--;
//    }
//
//    /* send last byte and a STOP */
//    I2C3_MDR_R = *data++;                 /* write the last byte */
//    I2C3_MCS_R = 5;                       /* -data-ACK-P */
//    error = I2C_wait_till_done();
//    while(I2C3_MCS_R & 0x40);             /* wait until bus is not busy */
//    if (error) return error;
//    return 0;       /* no error */
////    if (I2C3_MCS_R & 0x01) { // Check for busy state
////        I2C3_MCS_R = 0x04;   // Issue STOP condition to reset the bus
////        delayMs(1);          // Allow time for the bus to stabilize
////    }
////
////    // Wait for bus to be free
////    while (I2C3_MCS_R & 0x01); // ISSUE: I2C0_MCS_R is 32 here, triggering fault
////
////    I2C3_MSA_R = (EEPROM_ADDRESS << 1); // Send control byte with R/W = 0 (write mode) // 0b1010000 for 24LC32A
////    I2C3_MDR_R = (address >> 8); // & 0xFF; // Send high byte
////    I2C3_MCS_R = 0x03;                  // Start and run // CHANGES HERE
////    while (I2C3_MCS_R & 0x01);          // Wait for completion // ISSUE: I2C0_MCS_R is 48 here
////    if (I2C3_MCS_R & 0x02) {            // Check for errors
////        I2C0_MCS_R = 0x04;
////        return;                         // Exit if error
////    }
////
////    // Send low byte of address
////    I2C3_MDR_R = address & 0xFF;        // Low byte
////    I2C3_MCS_R = 0x01;                  // Run
////    while (I2C3_MCS_R & 0x01);          // Wait for completion
////    if (I2C3_MCS_R & 0x02) {            // Check for errors
////        I2C0_MCS_R = 0x04;
////        return;                         // Exit if error
////    }
////
////    // Send data byte
////    I2C3_MDR_R = data;                  // Data byte
////    I2C3_MCS_R = 0x05;                  // Run and stop
////    while (I2C3_MCS_R & 0x01);          // Wait for completion
////    if (I2C3_MCS_R & 0x02) {            // Check for errors
////        I2C0_MCS_R = 0x04;
////        return;                         // Exit if error
////    }
////
////    // Acknowledge polling to ensure write completion
////    do {
////        I2C3_MSA_R = (EEPROM_ADDRESS << 1); // Write mode
////        I2C3_MCS_R = 0x03;                  // Start and stop
////        while (I2C3_MCS_R & 0x01);          // Wait
////    } while (I2C3_MCS_R & 0x02);            // Retry if NACK received
//}


// Read data from EEPROM
char* readEEPROM(uint8_t address, uint8_t bytes_count) {
    if (bytes_count == 0) return 'E'; // No data to read

    static char data[1];

    while (I2C3_MCS_R & 0x01); // Wait for I2C bus to be idle

    // Write memory address
    I2C3_MSA_R = EEPROM_ADDRESS << 1; // R/W = 0
    I2C3_MDR_R = (address >> 8) & 0xFF; // High byte
    I2C3_MCS_R = 0x03; // START + RUN
    if (I2C_wait_till_done()) return 'E';

    I2C3_MDR_R = address & 0xFF; // Low byte
    I2C3_MCS_R = 0x01; // RUN
    if (I2C_wait_till_done()) return 'E';

    // Read data
    I2C3_MSA_R = (EEPROM_ADDRESS << 1) | 1; // R/W = 1
    int i;
    for (i = 0; i < bytes_count; i++) {
        I2C3_MCS_R = (i == bytes_count - 1) ? 0x07 : 0x09; // STOP for last byte
        if (I2C_wait_till_done()) return 'E';
        data[i] = I2C3_MDR_R; // Read byte
    }

    while (I2C3_MCS_R & 0x40); // Wait for bus to be idle
    return data;
}

//char* readEEPROM(uint8_t address, uint8_t bytes_count) {
//    char error;
//    char* data = malloc(bytes_count);
//    if (bytes_count <= 0)
//        return -1;         /* no read was performed */
//
//    /* send slave address and starting address */
//    I2C3_MSA_R = EEPROM_ADDRESS << 1;
//    I2C3_MDR_R = address;
//    I2C3_MCS_R = 3;       /* S-(saddr+w)-ACK-maddr-ACK */
//    error = I2C_wait_till_done();
//    if (error)
//        return error;
//
//    /* to change bus from write to read, send restart with slave addr */
//    I2C3_MSA_R = (EEPROM_ADDRESS << 1) + 1;   /* restart: -R-(saddr+r)-ACK */
//
//    if (bytes_count == 1)             /* if last byte, don't ack */
//        I2C3_MCS_R = 7;              /* -data-NACK-P */
//    else                            /* else ack */
//        I2C3_MCS_R = 0xB;            /* -data-ACK- */
//    error = I2C_wait_till_done();
//    if (error) return error;
//
//    *data++ = I2C3_MDR_R;            /* store the data received */
//
//    if (--bytes_count == 0)           /* if single byte read, done */
//    {
//        while(I2C3_MCS_R & 0x40);    /* wait until bus is not busy */
//        return 0;       /* no error */
//    }
//
//    /* read the rest of the bytes */
//    while (bytes_count > 1)
//    {
//        I2C3_MCS_R = 9;              /* -data-ACK- */
//        error = I2C_wait_till_done();
//        if (error) return error;
//        bytes_count--;
//        *data++ = I2C3_MDR_R;        /* store data received */
//    }
//
//    I2C3_MCS_R = 5;                  /* -data-NACK-P */
//    error = I2C_wait_till_done();
//    *data = I2C3_MDR_R;              /* store data received */
//    while(I2C3_MCS_R & 0x40);        /* wait until bus is not busy */
//
//    return data;       /* no error */
////    // Wait if I2C is busy
////    while (I2C3_MCS_R & 0x01);
////
////    // Send control byte with R/W = 0 (write mode)
////    I2C3_MSA_R = (EEPROM_ADDRESS << 1);
////
////    // Send high byte of address
////    I2C3_MDR_R = (address >> 8); //& 0xFF;
////    I2C3_MCS_R = 0x03; // Start and run
////    while (I2C3_MCS_R & 0x01); // Wait for completion
////
////    // Send low byte of address
////    I2C3_MDR_R = address; // & 0xFF;
////    I2C3_MCS_R = 0x05; // Run
////    while (I2C3_MCS_R & 0x01); // Wait for completion
////
////    // Send control byte with R/W = 1 (read mode)
////    I2C3_MSA_R = (EEPROM_ADDRESS << 1) | 1;
////    I2C3_MCS_R = 0x07; // Start, run, stop
////    while (I2C3_MCS_R & 0x01); // Wait for completion
////
////    I2C3_MCS_R = 0x05; // Run
////    while (I2C3_MCS_R & 0x01); // Wait for completion
////
////    return I2C3_MDR_R; // Read data
//}


void clearEEPROM() {
    int i;
    for (i = 0; i < MAX_NOTES; i++) {
        writeEEPROM(i, 0x00, 4); // Erase stored data
    }
}


// Get key from keypad
char getKey() {
    // Implement keypad scanning
    int row, col;

    GPIO_PORTA_DATA_R |= ROW_MASK;

    for (row = 0; row < ROWS; row++) {
        GPIO_PORTA_DATA_R |= ROW_MASK;
        GPIO_PORTA_DATA_R &= ~(1U << (2 + row));
        delayMs(10);

        if (!(GPIO_PORTA_DATA_R & (1U << 6))) {
            delayMs(20);
            if (!(GPIO_PORTA_DATA_R & (1U << 6))) {
                while (!(GPIO_PORTA_DATA_R & (1U << 6)));
                return keypad[row][0];
            }
        }
        if (!(GPIO_PORTA_DATA_R & (1U << 7))) {
            delayMs(20);
            if (!(GPIO_PORTA_DATA_R & (1U << 7))) {
                while (!(GPIO_PORTA_DATA_R & (1U << 7)));
                return keypad[row][1];
            }
        }

        if (!(GPIO_PORTC_DATA_R & (1U << 6))) {
            delayMs(20);
            if (!(GPIO_PORTC_DATA_R & (1U << 6))) {
                while (!(GPIO_PORTC_DATA_R & (1U << 6)));
                return keypad[row][2];
            }
        }
        if (!(GPIO_PORTC_DATA_R & (1U << 7))) {
            delayMs(20);
            if (!(GPIO_PORTC_DATA_R & (1U << 7))) {
                while (!(GPIO_PORTC_DATA_R & (1U << 7)));
                return keypad[row][3];
            }
        }
    }

    return 0;
}
