/*
 * SmartLeds.cpp
 *
 *  Created on: Jul 1, 2018
 *      Author: stan
 */

#include <memory>
#include <cassert>

#if defined ( ARDUINO )
    extern "C" { // ...someone forgot to put in the includes...
        #include "esp32-hal.h"
        #include "esp_intr.h"
        #include "driver/gpio.h"
        #include "driver/periph_ctrl.h"
        #include "freertos/semphr.h"
        #include "soc/rmt_struct.h"
        #include <driver/spi_master.h>
    }
#elif defined ( ESP_PLATFORM )
    extern "C" { // ...someone forgot to put in the includes...
        #include <esp_intr.h>
        #include <driver/gpio.h>
        #include <freertos/FreeRTOS.h>
        #include <freertos/semphr.h>
        #include <soc/dport_reg.h>
        #include <soc/gpio_sig_map.h>
        #include <soc/rmt_struct.h>
        #include <driver/spi_master.h>
    }
    #include <stdio.h>
#endif

#include "SmartLeds.h"



void SmartLed::copyRmtHalfBlock() {
        int offset = 0 ;// detail::MAX_PULSES * _halfIdx;
        int len = 3 - _componentPosition + 3 * ( _count - 1 );
        len = std::min( len, detail::MAX_PULSES / 8 );

        // Whole RMTMEM 512 indexs of 32bit
        uint32_t * RMTMEM_data32_val = (uint32_t *) &RMTMEM;

#define RMTMEM_IDX_CHK(idx)	((idx)<512?(idx):511)
#define RMTMEM_IDX(channel,idx)	( (channel*64) + idx)

        if ( !len ) {
            for ( int i = 0; i < detail::MAX_PULSES; i++) {
                RMTMEM.chan[_channel + _halfIdx].data32[i + offset].val = 0;
            }
        }

        int i;
        for ( i = 0; i != len && _pixelPosition != _count; i++ ) {
            uint8_t val = _buffer[ _pixelPosition ].getGrb( _componentPosition );
            for ( int j = 0; j != 8; j++, val <<= 1 ) {
                int bit = val >> 7;
                int idx = i * 8 + offset + j;
                RMTMEM.chan[ _channel + _halfIdx ].data32[ idx ].val = _bitToRmt[ bit & 0x01 ].value;
            }
            if ( _pixelPosition == _count - 1 && _componentPosition == 2 ) {
                RMTMEM.chan[ _channel + _halfIdx  ].data32[ i * 8 + offset + 7 ].duration1 =
                    _timing.TRS / ( detail::RMT_DURATION_NS * detail::DIVIDER );
            }

            _componentPosition++;
            if ( _componentPosition == 3 ) {
                _componentPosition = 0;
                _pixelPosition++;
            }
        }

        for ( i *= 8; i != detail::MAX_PULSES; i++ ) {
            RMTMEM.chan[ _channel + _halfIdx ].data32[ i + offset ].val = 0;
        }
        _halfIdx = !_halfIdx;
    }
