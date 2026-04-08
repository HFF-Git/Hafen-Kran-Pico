//----------------------------------------------------------------------------------------
//
// Harbor Application for Crane Movement Control
//
//----------------------------------------------------------------------------------------
// This application controls the movement of cranes in a set of harbors. The application
// uses a number of servos to control the cranes and a number of inputs to detect enable
// activity in the barbors. The application is designed to run on Arduino IDE-compatible
// microcontrollers and uses the Servo library for servo control. 
//
// The application is structured into several sections: pin and system declarations,
// input system, servo system, crane state management, harbor state management, harbor
// behavior, servo update, console I/O setup, and the main loop. Each section is 
// responsible for a specific aspect of the application, and the main loop orchestrates
// the overall behavior by calling the appropriate functions in a continuous loop. 
// 
// The application provides a simple simulation of harbor operations, where cranes move
// to pick up and drop off cargo based on the presence of ships and trains, as well
// as random activation and movement patterns when the harbor is idle.
//
//----------------------------------------------------------------------------------------
//
// Harbor Application for Crane Movement Control
// Copyright (C) 2026 - 2026 Helmut Fieres
//
// This program is free software: you can redistribute it and/or modify it under 
// the terms of the GNU General Public License as published by the Free Software 
// Foundation, either version 3 of the License, or any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
// PARTICULAR PURPOSE.  See the GNU General Public License for more details. You 
// should have received a copy of the GNU General Public License along with this 
// program. If not, see <http://www.gnu.org/licenses/>.
//
//  GNU General Public License:  http://opensource.org/licenses/GPL-3.0
//
//----------------------------------------------------------------------------------------
#include <EEPROM.h>
#include <Servo.h>

//----------------------------------------------------------------------------------------
// Pin and System Declarations. We have a number of servos and inputs. The servo pins
// are connected to the cranes, while the input pins are connected to buttons and 
// contacts. The servo and input systems are initialized in the setup routine and 
// updated in the main loop. 
//
//----------------------------------------------------------------------------------------
const int NUM_HARBORS       = 4;
const int NUM_CRANES        = 8;   
const int CRANES_PER_HARBOR = 2;
const int NUM_INPUTS        = 5;

int       servoPins[ NUM_CRANES ] = { 13, 12, 11, 10, 9, 8, 7, 6 };
int       inputPins[ NUM_INPUTS ] = { 16, 5, 4, 3, 2 };

Servo     servos[ NUM_CRANES ];

const uint32_t SERVO_INTERVAL_US = 20000; // 20 ms ... tst
uint32_t lastServoUpdateUs = 0;

int lastServoValue[NUM_CRANES];
int lastServoWrite[NUM_CRANES];

//----------------------------------------------------------------------------------------
// Input System Declarations. We have a number of inputs, which are connected to 
// buttons or contacts. The input system handles debouncing and provides the current
// state and the pressed state of each input.
//
//----------------------------------------------------------------------------------------
bool            inputState[ NUM_INPUTS ];
bool            inputPressed[ NUM_INPUTS ];
bool            prevState[ NUM_INPUTS ];
bool            lastRawState[ NUM_INPUTS ];
uint32_t        lastChangeTime[ NUM_INPUTS ];
const uint32_t  debounceTime = 30;

enum NamedButtons : int {

    BUTTON_CALIB   = 0,
    CONTACT_IN_1   = 1,
    CONTACT_IN_2   = 2,
    CONTACT_IN_3   = 3,
    CONTACT_IN_4   = 4
};

//----------------------------------------------------------------------------------------
// Servo System Declarations. We have a number of servos, which are connected to the
// cranes. Each servo supports up to four positions. The positions can be configured 
// using the calibration routine. 
//
//----------------------------------------------------------------------------------------
struct CranePositions {

    uint16_t  pos1[ NUM_CRANES ]   = { 40, 50, 60, 45, 40, 50, 60, 40  };
    uint16_t  pos2 [NUM_CRANES ]   = { 140,130,120,135, 140,130,120,135 };
    uint16_t  pos3[ NUM_CRANES ]   = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uint16_t  pos4[ NUM_CRANES ]   = { 0, 0, 0, 0, 0, 0, 0, 0 };
    float     speed[ NUM_CRANES ]  = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
};

CranePositions cranePositions;

//----------------------------------------------------------------------------------------
// Harbor Declarations. A harbor can be in one of two states: HARBOR_IDLE and
// HARBOR_ACTIVE. Each harbor has two cranes. We keep it in an array of four harbors
// with two cranes. The harbor data structure records the state and a timestamp for 
// time related harbor for eadh harbor.
//
//----------------------------------------------------------------------------------------
enum HarborState { 
  
    HARBOR_IDLE, 
    HARBOR_ACTIVE 
};

int harborCranes[ NUM_HARBORS ][ CRANES_PER_HARBOR ] = {
  
    { 0, 1 },   // Harbor 0
    { 2, 3 },   // Harbor 1
    { 4, 5 },   // Harbor 2
    { 6, 7 }    // Harbor 3
};

struct HarborData {

    HarborState state;
    uint32_t    stateUntil;

    int cranes[ CRANES_PER_HARBOR ];
};

HarborData harborData[ NUM_HARBORS ] = {

    { HARBOR_IDLE, 0, { 0, 1 }},
    { HARBOR_IDLE, 0, { 2, 3 }},
    { HARBOR_IDLE, 0, { 4, 5 }},
    { HARBOR_IDLE, 0, { 6, 7 }}
};

//----------------------------------------------------------------------------------------
// Crane State Declarations. Each crane can be in one of the following states: IDLE, 
// MOVE_TO_PICKUP, LOAD, MOVE_TO_DROPOFF, UNLOAD. The harbor behavior updates the 
// state of each crane and sets the target position accordingly. The stateUntil field
// is used to implement timed behavior, e.g. the crane stays in the LOAD state for a 
// certain time before moving to the next state. 
//
//----------------------------------------------------------------------------------------
enum CraneState { 
  
    IDLE, 
    MOVE_TO_PICKUP, 
    LOAD, 
    MOVE_TO_DROPOFF, 
    UNLOAD 
};

struct Crane {

    // --- State machine ---
    uint8_t  state;
    uint32_t stateUntil;

    // --- Motion ---
    float    pos;        // current position
    float    startPos;
    float    targetPos;

    uint32_t startTimeUs;
    uint32_t durationUs;
    bool     moving;
};

Crane crane[ NUM_CRANES ];

//----------------------------------------------------------------------------------------
// Little helper function to make sure that the servo positions are within a reasonable
// range. 
//
//----------------------------------------------------------------------------------------
int clamp( int arg, int low, int high ) {

    if ( arg < low ) arg = low;
    if ( arg > high ) arg = high;
    return( arg );
}

float clamp( float arg, float low, float high ) {

    if ( arg < low ) arg = low;
    if ( arg > high ) arg = high;
    return( arg );
}

//----------------------------------------------------------------------------------------
// Print the crane positions. The routine is used in the calibration process to
// show the current positions of each crane.
//
//----------------------------------------------------------------------------------------
void printCranePositions( int i ) {

    Serial.print( "Crane " );
    Serial.print( i );
    Serial.print(": Pos1: " );
    Serial.print( cranePositions.pos1[ i ] );
    Serial.print(": Pos2: " );
    Serial.print( cranePositions.pos2[ i ] );
    Serial.print(": Pos3: " );
    Serial.print( cranePositions.pos3[ i ] );
    Serial.print(": Pos4: " );
    Serial.print( cranePositions.pos4[ i ] );
    Serial.print(": Speed: " );
    Serial.print( cranePositions.speed[ i ] );
    Serial.println( );
}

//----------------------------------------------------------------------------------------
// Initialize the EEPROM facility. The EEPROM stores the crane data. We need to pass 
// the size of the used storage. The PICO does not have an EEPROM, so it uses some 
// part of flash for this purpose. Note that the nuber of flash writes are very limited,
// so we should not do this very often.
//
//----------------------------------------------------------------------------------------
void setupEEPROM( ) {

    EEPROM.begin( sizeof( cranePositions ));
}

//----------------------------------------------------------------------------------------
// Load crane settings from EEPROM. The routine reads the crane settings from the
// EEPROM and updates the cranePos arrays. The routine is called in the setup process 
// to initialize the crane positions. The routine assumes that the crane settings are 
// stored in a specific format in the EEPROM, with each crane's settings stored 
// sequentially. 
//
//----------------------------------------------------------------------------------------
void loadCraneSettingsFromEEPROM( ) {

    Serial.println( "Load Crane EEPROM settings" );

    EEPROM.get( 0, cranePositions );
    
    for ( int i = 0; i < NUM_CRANES; i++ ) {

        cranePositions.pos1[ i ]  = clamp( cranePositions.pos1[ i ], 1000, 2000 );
        cranePositions.pos2[ i ]  = clamp( cranePositions.pos2[ i ], 1000, 2000 );
        cranePositions.pos3[ i ]  = clamp( cranePositions.pos3[ i ], 1000, 2000 );
        cranePositions.pos4[ i ]  = clamp( cranePositions.pos4[ i ], 1000, 2000 );

        if ( isfinite( cranePositions.speed[ i ] )) 
            cranePositions.speed[ i ] = clamp( cranePositions.speed[ i ], 0.1, 5.0 );
        else
            cranePositions.speed[ i ] = 1.0;

        printCranePositions( i );
    }
}

//----------------------------------------------------------------------------------------
// Update crane settings in EEPROM. The routine writes the current crane settings to
// the EEPROM. The routine is called after the calibration process to save the new 
// settings. The PICO does not have an EEPROM, so it uses some part of flash for this
// purpose. Note that the nuber of flash writes are very limited, so we should not do
// this very often.
//
//----------------------------------------------------------------------------------------
void updateCraneSettingsInEEPROM( ) {

    Serial.println( "Update Crane EEPROM settings" );

    EEPROM.put( 0, cranePositions );
    EEPROM.commit( );
}

//----------------------------------------------------------------------------------------
// Read the input line for the calibrate command. The routine returns the number of
// characters read. If no console is available, it returns immediately with zero. The 
// routine waits for a full  line of input, which is terminated by a newline character.
// The carriage return 
//
//----------------------------------------------------------------------------------------
int readLine( String &out ) {

    if (!Serial) return ( 0 );

    out = "";

    while ( true ) {

        while ( Serial.available( )) {

            char c = Serial.read( );

            if ( c == '\n' ) return ( out.length( ));
            if ( c != '\r' ) out += c;
        }
    }
}

//----------------------------------------------------------------------------------------
// Parse the integer input. A crane has up to four positions. There are four values read.
// We also read in the speed value, which is a float. The routine returns the number of
// values read. If the input is not in the expected format, the routine returns -1. 
// The routine is used in the calibration process to read the new settings for a crane. 
// The expected input format is: "pos1 pos2 pos3 pos4 speed", where pos1 to pos4 are 
// integers and speed is a float.
//
//----------------------------------------------------------------------------------------
int readConfigArguments( int &a, int &b, int &c, int &d, float &speed ) {

    String line;

    if ( readLine( line ) == 0 ) return ( 0 );

    int tmpA, tmpB, tmpC, tmpD;
    float tmpSpeed;
    int count = sscanf( line.c_str( ), "%d %d %d %d %f", 
                        &tmpA, &tmpB, &tmpC, &tmpD, &tmpSpeed );

    if ( count == 5 ) {
    
        a       = tmpA;
        b       = tmpB;
        c       = tmpC;
        d       = tmpD;
        speed   = tmpSpeed;
        return ( count );
    }

    return ( -1 );
}

//----------------------------------------------------------------------------------------
// Demo settings for a crane. The routine moves the crane through the four positions with
// a delay in between. The routine is used to demonstrate the effect of the crane settings
// and to verify that the crane is working correctly. The routine can be called from the
// calibration process to show the effect of the new settings. We will use the normal
// crane update routine to move the crane to the target position, which allows us to see
// the movement speed and behavior based on the harbor state. The routine waits for the
// servo to reach each target position before moving to the next one, providing a clear
// demonstration of the servo settings.
//
//----------------------------------------------------------------------------------------
void demoCraneSettings( int i ) {

    Serial.print( "Demo settings for Crane" );
    Serial.println( i );

    moveServo( i, cranePositions.pos1[ i ] );
    while ( crane[ i ].moving ) handleServos( );
    delay( 1000 );

    moveServo( i, cranePositions.pos2[ i ] );
    while ( crane[ i ].moving) handleServos( );
    delay( 1000 );

    moveServo( i, cranePositions.pos3[ i ] );
    while (crane[ i ].moving) handleServos( );
    delay( 1000 );

    moveServo(  i, cranePositions.pos4[ i ] );
    while ( crane[ i ].moving) handleServos();
    delay( 1000 );
}

//----------------------------------------------------------------------------------------
// Calibration processing for a servo. The routine is a bit crude. We list the current 
// setting and then prompt for each servo. For each servo either four values are entered
// or just a carriage return. In the latter case, the current values set are retained 
// and we move to the next servo. 
//
//----------------------------------------------------------------------------------------
void calibrate( ) {

    Serial.println( "Crane Position Calibration mode" );
    Serial.println( );

    for ( int i = 0; i < NUM_CRANES; i++ ) printCranePositions( i );
    Serial.println( );

    for ( int i = 0; i < NUM_CRANES; i++ ) {
    
        Serial.print("Crane ");
        Serial.print( i );
    
        while ( true ) {
    
            int newPos1, newPos2, newPos3, newPos4, cnt;
            float newSpeed;
        
            Serial.print(" -> ");
            cnt = readConfigArguments( newPos1, newPos2, newPos3, newPos4, newSpeed );
        
            if ( cnt == 0 ) {

                break;
            }
            else if ( cnt == 5 ) {

                cranePositions.pos1[ i ]  = clamp( newPos1, 1000, 2000 );
                cranePositions.pos2[ i ]  = clamp( newPos2, 1000, 2000 );    
                cranePositions.pos3[ i ]  = clamp( newPos3, 1000, 2000 );
                cranePositions.pos4[ i ]  = clamp (newPos4, 1000, 2000 );
                cranePositions.speed[ i ] = newSpeed;

                demoCraneSettings( i );
            } 
            else Serial.println( "Expected four integers and one float" );
        }
    
        Serial.println( );
    }

    updateCraneSettingsInEEPROM( );
    loadCraneSettingsFromEEPROM( );
}

//----------------------------------------------------------------------------------------
// Setup input processing. We configure the input pins as input with pull-up resistors.
// All inputs are debounced and stored as state changed. 
//
//----------------------------------------------------------------------------------------
void setupInputs( ) {
  
    for ( int i = 0; i < NUM_INPUTS; i++ ) {
        
        pinMode( inputPins[ i ], INPUT_PULLUP );

        bool raw = ( digitalRead( inputPins[ i ] ) == LOW );

        inputState[ i ]     = raw;
        prevState[ i ]      = raw;
        lastRawState[ i ]   = raw;
        lastChangeTime[ i ] = millis();
    }
}

//----------------------------------------------------------------------------------------
// Update input states. We record the input pressed state, which is used by the upper 
// layers. The input state is updated based on the raw input and the debounce time. 
// The pressed state is set when the input state changes from not pressed to pressed.
//
//----------------------------------------------------------------------------------------
void updateInputs( ) {
  
    uint32_t now = millis( );

    for ( int i = 0; i < NUM_INPUTS; i++ ) {

        bool raw = ( digitalRead( inputPins[ i ] ) == LOW );

        if ( raw != lastRawState[ i ]) {
        
            lastChangeTime[ i ] = now;
            lastRawState[ i ]   = raw;
        }

        if (( now - lastChangeTime[ i ] ) > debounceTime ) inputState[ i ] = raw;

        inputPressed[ i ] = ( ! prevState[ i ] && inputState[ i ]);
        prevState[ i ]    = inputState[ i ];
    }
}

//----------------------------------------------------------------------------------------
// Handle input events. The routine is called periodically and the input pressed array
// contains the information whether a contact was closed or button was pressed. The
// contact inputs are used to toggle the harbor state to active or inactive. The button
// input is used to trigger crane position calibration.
//
//----------------------------------------------------------------------------------------
void handleInputs( ) {

    if ( inputPressed[ BUTTON_CALIB ] ) calibrate( );

    if ( inputPressed[ CONTACT_IN_1 ] ) {

        Serial.println( "Harbor 0 Active" );
        harborData[ 0 ].state = ( harborData[ 0 ].state == HARBOR_ACTIVE ) ? 
                                HARBOR_ACTIVE : HARBOR_IDLE;
        harborData[ 0 ].stateUntil = millis( ) + 60 * 1000;
    }

    if ( inputPressed[ CONTACT_IN_2 ] ) {

        Serial.println( "Harbor 1 Active" );
        harborData[ 1 ].state = ( harborData[ 1 ].state == HARBOR_ACTIVE ) ? 
                                HARBOR_ACTIVE : HARBOR_IDLE;
        harborData[ 1 ].stateUntil = millis( ) + 60 * 1000;
    }

    if ( inputPressed[ CONTACT_IN_3 ] ) {
        
        Serial.println( "Harbor 2 Active" );
        harborData[ 2 ].state = ( harborData[ 2 ].state == HARBOR_ACTIVE ) ? 
                                HARBOR_ACTIVE : HARBOR_IDLE;
        harborData[ 2 ].stateUntil = millis( ) + 60 * 1000;
    }

    if ( inputPressed[ CONTACT_IN_4 ] ) {
        
        Serial.println( "Harbor 3 Active" );
        harborData[ 3 ].state = ( harborData[ 3 ].state == HARBOR_ACTIVE ) ? 
                                HARBOR_ACTIVE : HARBOR_IDLE;
        harborData[ 3 ].stateUntil = millis( ) + 60 * 1000;
    }
}

//----------------------------------------------------------------------------------------
// Update the servos. The routine is called periodically from within the main loop. 
// The current position of each servo a is updated based on the target position and 
// time to get there. 
//
// ??? really look into this .....
//----------------------------------------------------------------------------------------
void handleServos( ) {

    uint32_t now = micros();

    for ( int i = 0; i < NUM_CRANES; i++ ) {

        Crane &c = crane[ i ];

        if ( ! c.moving ) continue;

        float t = (float)( now - c.startTimeUs ) / (float) c.durationUs;

        if ( t >= 1.0f ) {

            t = 1.0f;
            c.moving = false;
        }

        t = t * t * ( 3.0f - 2.0f * t ); // easing

        float newPos = c.startPos + ( c.targetPos - c.startPos ) * t;

        c.pos = newPos;

        int out = (int)( newPos + 0.5f );

        if ( now - lastServoWrite[ i ] >= 5000 ) {   // 5ms = 200Hz max update

            if ( abs( out - lastServoValue[ i ]) >= 2 ) {

                servos[ i ].writeMicroseconds( out );
                lastServoValue[ i ] = out;
            }

            lastServoWrite[ i ] = now;
        }
    }
}

//----------------------------------------------------------------------------------------
// Move the servo to the target position. The routine calculates the distance to the 
// target position and moves the servo accordingly. If the distance is small, the routine
// returns immediately. The movement speed is based on the crane settings and is 
// randomized a bit to create a more natural movement. The duration of the movement is
// calculated based on the distance and the speed, and is clamped to a reasonable range. 
// The routine updates the crane state to indicate that the crane is moving, and records
// the start time and duration of the movement. The routine is used by the harbor behavior
// to set the move parameters for the crane to the pickup and dropoff positions based 
// on the current state of the crane.
//
//----------------------------------------------------------------------------------------
void moveServo( int i, float target ) {

    Crane &c = crane[ i ];

    float distance = abs( target - c.pos );
    if ( distance < 1.0f ) return;

    float speed = cranePositions.speed[ i ];
    speed *= random( 90, 110 ) / 100.0f;

    uint32_t durationMs = (uint32_t)( distance / speed );
    durationMs = clamp( durationMs, 500, 8000 );

    c.startPos    = c.pos;
    c.targetPos   = target;
    c.startTimeUs = micros();
    c.durationUs  = durationMs * 1000;
    c.moving      = true;
}

//----------------------------------------------------------------------------------------
// Crane behavior. The crane behavior updates the state of each crane and sets the target
// position accordingly. The stateUntil field is used to implement timed behavior, e.g. 
// the crane stays in the LOAD state for a certain time before moving to the next state.
// The activation chance and speed of the cranes depend on the harbor state. When the
// harbor is active, the cranes are more likely to activate and move faster. When the
// harbor is idle, the cranes are less likely to activate and move slower. The crane
// behavior is called periodically from the harbor behavior to update the state of each
// crane based on the current harbor state and the crane's current state. The behavior
// implements a simple state machine for each crane, where the crane transitions between
// the IDLE, MOVE_TO_PICKUP, LOAD, MOVE_TO_DROPOFF, and UNLOAD states based on the 
// current state and the harbor state. The behavior also uses random timers to create
// a more natural and dynamic behavior for the cranes.
//
//----------------------------------------------------------------------------------------
void craneBehavior( int harbor, int i ) {

    Crane    &c  = crane[i];
    uint32_t now = millis();

    if ( now < c.stateUntil ) return;

    int activationChance = ( harborData[ harbor ].state == HARBOR_ACTIVE)  ? 70 : 15;

    switch ( c.state ) {

        case IDLE: {

            if ( random(100) < activationChance ) {

                c.state = MOVE_TO_PICKUP;

                float target = cranePositions.pos1[ i ] + random( -20, 40 );
                moveServo( i, target );

            } else c.stateUntil = now + random(2000, 6000);
            
        } break;

        case MOVE_TO_PICKUP: {

            if ( ! c.moving ) {

                c.state      = LOAD;
                c.stateUntil = now + random( 2000, 4000 );
            }

        } break;

        case LOAD: {

            c.state = MOVE_TO_DROPOFF;

            float target = cranePositions.pos2[ i ] + random( -3, 4 );
            moveServo(i, target);
        
        } break;

        case MOVE_TO_DROPOFF: {

            if ( ! c.moving ) {

                c.state      = UNLOAD;
                c.stateUntil = now + random(2000, 4000);
            }

        } break;

        case UNLOAD: {

            c.state      = IDLE;
            c.stateUntil = now + random(3000, 8000);
        
        } break;
    }
}

//----------------------------------------------------------------------------------------
// Harbor behavior. The harbor behavior cycles through each harbor and invokes the crane
// behavior routine.
//
//----------------------------------------------------------------------------------------
void harborBehavior() {

    for ( int h = 0; h < NUM_HARBORS; h++ ) {

        for ( int j = 0; j < CRANES_PER_HARBOR; j++)  {

            int c = harborCranes[ h ][ j ];
            craneBehavior( h, c );
        }
    }
}

//----------------------------------------------------------------------------------------
// Setup console I/O. We initialize the serial port for debugging and optional user 
// interaction. The routine waits for the serial port to be ready and then prints a 
// welcome message. The console I/O is used for calibration and debugging purposes.
// While we are at it, we also seed the random number facility.
//
//----------------------------------------------------------------------------------------
void setupConsoleIo( ) {

    Serial.begin( 115200 );  
    
    uint32_t start = millis( ); 
    while (( ! Serial ) && ( millis( ) - start < 2000 )) delay( 10 );

    randomSeed( analogRead( 26 ));

    Serial.println( "Harbor Application" );
    Serial.println( );
}

//----------------------------------------------------------------------------------------
// Initialize the cranes. We configure the crane using the digital I/O pin where the
// crane can be found. We will set the initial position to the "pos1" value from the
// crane data loaded from the EEPROM. 
//
//----------------------------------------------------------------------------------------
void setupCranes( ) {

    for ( int i = 0; i < NUM_CRANES; i++ ) {

        servos[ i ].attach(servoPins[ i ]);

        crane[ i ].state        = IDLE;
        crane[ i ].pos          = cranePositions.pos1[ i ];
        crane[ i ].targetPos    = crane[ i ].pos;
        crane[ i ].moving       = false;

        crane[ i ].startTimeUs  = micros( );
      //  crane[ i ].lastUpdateUs = crane[ i ].startTimeUs;  // ??? impact ?
        crane[ i ].stateUntil   = millis() + random( 1000, 5000 );

        int val = (int)crane[ i ].pos;

        servos[ i ].writeMicroseconds( val );
        lastServoValue[ i ] = val; 
    }
}

//----------------------------------------------------------------------------------------
// The setup routine initializes the console I/O, the input system, and the servo 
// sysrems. It also initializes the state of each crane and the harbor. The cranes 
// start in the IDLE state with a target position of 90 degrees. The harbor starts 
// in the HARBOR_IDLE state. It also initializes the state of each crane and the 
// harbor.
//
//----------------------------------------------------------------------------------------
void setup( ) {
  
    setupConsoleIo( );
    setupEEPROM( );
    setupInputs( );
    loadCraneSettingsFromEEPROM( );
    setupCranes( );
}

//----------------------------------------------------------------------------------------
// Main Loop. The main loop updates the input states, handles input events, updates 
// the harbor behavior, and sets the servo positions. The loop runs continuously, 
// providing real-time control of the cranes based on the presence of ships and 
// trains, as well as timed behavior. 
//
//----------------------------------------------------------------------------------------
void loop() {

    updateInputs( );
    handleInputs( );
    harborBehavior( );
    handleServos( );
}
