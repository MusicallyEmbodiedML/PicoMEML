#include "hardware/adc.h"
extern "C" {
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include "lwip/apps/httpd.h"
}
#include "ssi.h"
#include "cgi.h"
#include <sstream>
#include <iomanip> // For std::fixed and std::setprecision

#include "../common/common_defs.h"
#include "../PicoDefs.hpp"


const char * __not_in_flash("httpd") ssi_example_tags[] = {
    "NITER", // 0
    "LERR",  // 1
    "EXRG",  // 2
    "APID",  // 3
    "CDAT",  // 4
    "CMOD",  // 5
    "NNMD",  // 6
    "EXMD"   // 7
};


u16_t __time_critical_func(ssi_handler)(int iIndex, char *pcInsert, int iInsertLen) {
    if (pcInsert == nullptr || iInsertLen <= 0) {
        return 0; // Invalid parameters, nothing written
    }

    switch (iIndex) {
        case 0: // "NITER"
            return snprintf(pcInsert, iInsertLen, "%u", gAppState.n_iterations);
        case 1: // "LERR"
            return snprintf(pcInsert, iInsertLen, "%.6f", gAppState.last_error);
        case 2: // "EXRG"
            return snprintf(pcInsert, iInsertLen, "%.6f", gAppState.exploration_range);
        case 3: // "APID"
            return snprintf(pcInsert, iInsertLen, "%d", static_cast<int>(gAppState.app_id));
        case 4: // "CDAT"
            return snprintf(pcInsert, iInsertLen, "%d", static_cast<int>(gAppState.current_dataset));
        case 5: // "CMOD"
            return snprintf(pcInsert, iInsertLen, "%d", static_cast<int>(gAppState.current_model));
        case 6: // "NNMD"
            return snprintf(pcInsert, iInsertLen, "%d", static_cast<int>(gAppState.current_nn_mode));
        case 7: // "EXMD"
            return snprintf(pcInsert, iInsertLen, "%d", static_cast<int>(gAppState.current_expl_mode));
        default:
            return 0; // Unknown index, nothing written
    }
}


void ssi_init()
{
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    for (size_t i = 0; i < LWIP_ARRAYSIZE(ssi_example_tags); i++) {
        LWIP_ASSERT("tag too long for LWIP_HTTPD_MAX_TAG_NAME_LEN",
                    strlen(ssi_example_tags[i]) <= LWIP_HTTPD_MAX_TAG_NAME_LEN);
    }

      http_set_ssi_handler(ssi_handler,
                           ssi_example_tags, LWIP_ARRAYSIZE(ssi_example_tags)
      );
}
