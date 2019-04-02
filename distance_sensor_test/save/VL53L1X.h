/*
  This is a library written for the VL53L1X
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14667

  Written by Nathan Seidle @ SparkFun Electronics, April 12th, 2018

  The VL53L1X is the latest Time Of Flight (ToF) sensor to be released. It uses
  a VCSEL (vertical cavity surface emitting laser) to emit a class 1 IR laser
  and time the reflection to the target. What does all this mean? You can measure
  the distance to an object up to 4 meters away with millimeter resolution!

  We’ve found the precision of the sensor to be 1mm but the accuracy is around +/-5mm.

  This library handles the initialization of the VL53L1X and is able to query the sensor
  for different readings.

  Because ST has chosen not to release a complete datasheet we are forced to reverse
  engineer the interface from their example code and I2C data stream captures.
  For ST's reference code see STSW-IMG007

  The device operates as a normal I2C device. There are *many* registers. Reading and 
  writing happens with a 16-bit address. The VL53L1X auto-increments the memory
  pointer with each read or write.

  Development environment specifics:
  Arduino IDE 1.8.5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "vl53l1_register_map.h"

#include <stdint.h>
#include <stdbool.h>

class VL53L1X {

  public:

    bool     begin(uint8_t deviceAddress = defaultAddress_VL53L1X);
    void     softReset(); //Reset the sensor via software
    void     startMeasurement(uint8_t offset = 0); //Write a block of bytes to the sensor to configure it to take a measurement
    bool     newDataReady(); //Polls the measurement completion bit
    uint16_t getDistance(); //Returns the results from the last measurement, distance in mm
    uint16_t getSignalRate(); //Returns the results from the last measurement, signal rate
    void     setDistanceMode(uint8_t mode = 2);//Defaults to long range
    uint8_t  getDistanceMode();
    uint8_t  getRangeStatus(); //Returns the results from the last measurement, 0 = valid

  private:

    static const uint8_t defaultAddress_VL53L1X = 0x29; 
//    static constexpr uint8_t I2C_BUFFER_LENGTH = 32;
    static constexpr uint8_t I2C_BUFFER_LENGTH = 128;

    //Variables
    uint8_t _i2cport; //The generic connection to user's chosen I2C hardware
    uint8_t _distanceMode = 0;
};

