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

namespace detail {


 const int DIVIDER = 4; // 8 still seems to work, but timings become marginal
 const int MAX_PULSES = 192; // A channel has a 64 "pulse" buffer - we use half per pass
 const double RMT_DURATION_NS = 12.5; // minimum time of a single RMT duration based on clock ns

} // namespace detail



void SmartLed::copyRmtHalfBlock() {
        int len = 3 - _componentPosition + 3 * ( _count - 1 );
        len = std::min( len, detail::MAX_PULSES / 8 );

        // Whole RMTMEM 512 indexs of 32bit
        uint32_t * RMTMEM_data32_val = (uint32_t *) &RMTMEM;

#define RMTMEM_IDX_CHK(idx)	((idx)<512?(idx):511)
#define RMTMEM_IDX(channel,idx)	RMTMEM_IDX_CHK( (channel*64) + idx)

        if ( !len ) {
            for ( int i = 0; i < detail::MAX_PULSES; i++) {
                RMTMEM_data32_val[ RMTMEM_IDX(_channel,_halfIdx*detail::MAX_PULSES + i) ] = 0;
            }
        }

        int idx_pxl;
        for ( idx_pxl = 0; idx_pxl != len && _pixelPosition != _count; idx_pxl++ )
        {
            uint8_t pxl_cpnt_val = _buffer[ _pixelPosition ].getGrb( _componentPosition );
            for ( int idx_pxl_cpnt_bit = 0; idx_pxl_cpnt_bit != 8; idx_pxl_cpnt_bit++, pxl_cpnt_val <<= 1 )
            {
                RMTMEM_data32_val[ RMTMEM_IDX(_channel,_halfIdx*detail::MAX_PULSES + idx_pxl * 8 + idx_pxl_cpnt_bit ) ] = _bitToRmt[ !!(pxl_cpnt_val & 0x80) ].value;;
            }
            if ( _pixelPosition == _count - 1 && _componentPosition == 2 )
            {
            	// Last shape in the transfer
            	((rmt_item32_t*) &RMTMEM_data32_val[ RMTMEM_IDX(_channel,_halfIdx*detail::MAX_PULSES + idx_pxl * 8 + 7 ) ] )->duration1 =
                        _timing.TRS / ( detail::RMT_DURATION_NS * detail::DIVIDER );

//            	// Pulse following the last
//            	RMTMEM_data32_val[ RMTMEM_IDX(_channel,_halfIdx*detail::MAX_PULSES + idx_pxl * 8 + 8 ) ] = 0;
//            	// The first pulse of the channel
//            	RMTMEM_data32_val[ RMTMEM_IDX(_channel, 0 ) ] = 0;
            }

            _componentPosition++;
            if ( _componentPosition == 3 ) {
                _componentPosition = 0;
                _pixelPosition++;
            }
        }

        for ( idx_pxl *= 8; idx_pxl != detail::MAX_PULSES; idx_pxl++ ) {
        	RMTMEM_data32_val[ RMTMEM_IDX(_channel,_halfIdx*detail::MAX_PULSES + idx_pxl ) ] = 0;
        }
        _halfIdx = !_halfIdx;
    }
