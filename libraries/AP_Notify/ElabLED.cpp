/*
  Buzzer driver
*/
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "ElabLED.h"

#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;

bool ElabLED::init()
{
    // return immediately if disabled
    if (!AP_Notify::flags.external_leds) {
        return false;
    }


    return true;
}

// update - updates led according to timed_updated.  Should be called at 50Hz
void ElabLED::update()
{
    // return immediately if disabled
    if (!AP_Notify::flags.external_leds) {
        return;
    }

    // check for arming failed event
    if (AP_Notify::events.arming_failed) {
        SRV_Channels::set_output_pwm(SRV_Channel::k_ElabLED, 1500);
    }


}

