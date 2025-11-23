/*
  Copyright (C) 2013-2017 the Firmata Authors. See AUTHORS for more details.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

/*
 * Firmata is a generic protocol for communicating with microcontrollers
 * from software on a host computer. It is intended to work with
 * any host computer software package.
 *
 * To download a host software package, please visit http://firmata.org.
 */

#include <Servo.h>
#include <Firmata.h>

byte reportDigitalPorts = 0;

// Analog pin sampling
byte analogInputsToReport = 0;

// digital input pins
byte previousPIN[TOTAL_PORTS];

// digital output ports
byte portConfigInputs[TOTAL_PORTS];

// servos
Servo servos[MAX_SERVOS];
byte servoPinMap[MAX_SERVOS];
byte servoCount = 0;

// START CALLBACKS ==============================================================

void setPinModeCallback(byte pin, int mode) {
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT || mode == INPUT_PULLUP) {
      portConfigInputs[pin/8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin/8] &= ~(1 << (pin & 7));
    }
  }

  switch (mode) {
    case ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);
        }
        analogInputsToReport |= (1 << pin);
      }
      break;

    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT);
      }
      break;

    case INPUT_PULLUP:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);
      }
      break;

    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
      }
      break;

    case PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
      }
      break;

    case SERVO:
      if (servoCount < MAX_SERVOS) {
        servos[servoCount].attach(PIN_TO_DIGITAL(pin));
        servoPinMap[servoCount] = pin;
        servoCount++;
      }
      break;

    default:
      break;
  }

  Firmata.setPinMode(pin, mode);
}

void reportAnalogCallback(byte analogPin, int value) {
  if (value == 0) {
    analogInputsToReport &= ~(1 << analogPin);
  } else {
    analogInputsToReport |= (1 << analogPin);
    if (!IS_PIN_ANALOG(analogPin)) return;
  }
}

void reportDigitalCallback(byte port, int value) {
  if (port < TOTAL_PORTS) {
    if (value == 0) {
      reportDigitalPorts &= ~(1 << port);
    } else {
      reportDigitalPorts |= (1 << port);

      byte pin, lastPin = port * 8;
      for (pin = lastPin; pin < lastPin + 8 && pin < TOTAL_PINS; pin++) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);
        }
      }
    }
  }
}

void analogWriteCallback(byte pin, int value) {
  if (IS_PIN_PWM(pin)) {
    analogWrite(PIN_TO_PWM(pin), value);
  } else if (IS_PIN_SERVO(pin)) {
    for (byte i = 0; i < servoCount; i++) {
      if (servoPinMap[i] == pin) {
        servos[i].write(value);
      }
    }
  }
}

void digitalWriteCallback(byte pin, int value) {
  byte port = pin / 8;
  byte offset = pin - port * 8;

  if (value == 0) {
    Firmata.setDigitalPin(pin, 0);
    portConfigInputs[port] |= (1 << offset);
  } else {
    Firmata.setDigitalPin(pin, 1);
    portConfigInputs[port] &= ~(1 << offset);
  }

  writePort(port, Firmata.getDigitalPort(port));
}

void sysexCallback(byte command, byte argc, byte *argv) {
  switch (command) {
    case SERVO_CONFIG: {
        byte pin = argv[0];
        int minPulse = argv[1] + (argv[2] << 7);
        int maxPulse = argv[3] + (argv[4] << 7);

        if (servoCount < MAX_SERVOS) {
          servos[servoCount].attach(pin, minPulse, maxPulse);
          servoPinMap[servoCount] = pin;
          servoCount++;
        }
      }
      break;

    case STRING_DATA:
      break;
  }
}

// SETUP =======================================================================

void setup() {
  Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);

  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(START_SYSEX, sysexCallback);

  Firmata.begin(57600);

  for (byte port = 0; port < TOTAL_PORTS; port++) {
    previousPIN[port] = 0;
    portConfigInputs[port] = 0;
  }
}

// LOOP ========================================================================

void loop() {
  while (Firmata.available()) {
    Firmata.processInput();
  }

  for (byte analogPin = 0; analogPin < TOTAL_ANALOG_PINS; analogPin++) {
    if (analogInputsToReport & (1 << analogPin)) {
      Firmata.sendAnalog(analogPin, analogRead(analogPin));
    }
  }

  for (byte port = 0; port < TOTAL_PORTS; port++) {
    if (reportDigitalPorts & (1 << port)) {
      byte currentPIN = readPort(port, portConfigInputs[port]);
      if (currentPIN != previousPIN[port]) {
        Firmata.sendDigitalPort(port, currentPIN);
        previousPIN[port] = currentPIN;
      }
    }
  }
}
