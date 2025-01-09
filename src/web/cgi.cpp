#include "cgi.h"

extern "C" {
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include "lwip/apps/httpd.h"
}

#include "../common/common_defs.h"
#include "../PicoDefs.hpp"

#include <Arduino.h>


#define NUMCGIHANDLERS 5

static const tCGI cgi_handlers[NUMCGIHANDLERS] = {
    {
        /* Html request for "/leds.cgi" will start cgi_handler_basic */
        "/leds.cgi", cgi_handler_basic
    },
    {
        /* Html request for "/leds2.cgi" will start cgi_handler_extended */
        "/leds_ext.cgi", cgi_handler_extended
    }
,
    {
        "/dials.cgi", cgi_handler_dial
    },
    {
        "/buttons.cgi", cgi_handler_button
    },
    {
        "/poll.cgi", cgi_handler_poll
    },
};



/* cgi-handler triggered by a request for "/leds.cgi" */
const char *
cgi_handler_basic(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    int i=0;

    /* We use this handler for one page request only: "/leds.cgi"
     * and it is at position 0 in the tCGI array (see above).
     * So iIndex should be 0.
     */
    Serial.printf("cgi_handler_basic called with index %d\n", iIndex);

    /* All leds off */
    Led_Off(LED1);
    Led_Off(LED2);
    Led_Off(LED3);
    Led_Off(LED4);

    /* Check the query string.
     * A request to turn LED2 and LED4 on would look like: "/leds.cgi?led=2&led=4"
     */
    for (i = 0; i < iNumParams; i++){
        /* check if parameter is "led" */
        if (strcmp(pcParam[i] , "led") == 0){
            /* look ar argument to find which led to turn on */
            if(strcmp(pcValue[i], "1") == 0)
                Led_On(LED1);
            else if(strcmp(pcValue[i], "2") == 0)
                Led_On(LED2);
            else if(strcmp(pcValue[i], "3") == 0)
                Led_On(LED3);
            else if(strcmp(pcValue[i], "4") == 0)
                Led_On(LED4);
        }
    }

    /* Our response to the "SUBMIT" is to simply send the same page again*/
    return "/cgi.html";
}

/* cgi-handler triggered by a request for "/leds_ext.cgi".
 *
 * It is almost identical to cgi_handler_basic().
 * Both handlers could be easily implemented in one function -
 * distinguish them by looking at the iIndex parameter.
 * I left it this way to show how to implement two (or more)
 * enirely different handlers.
 */
const char *
cgi_handler_extended(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    int i=0;

    /* We use this handler for one page request only: "/leds_ext.cgi"
     * and it is at position 1 in the tCGI array (see above).
     * So iIndex should be 1.
     */
    Serial.printf("cgi_handler_extended called with index %d\n", iIndex);

    /* All leds off */
    Led_Off(LED1);
    Led_Off(LED2);
    Led_Off(LED3);
    Led_Off(LED4);

    /* Check the query string.
     * A request to turn LED2 and LED4 on would look like: "/leds.cgi?led=2&led=4"
     */
    for (i = 0; i < iNumParams; i++){
        if (strcmp(pcParam[i] , "v") == 0){
            Serial.printf("v: %s", pcValue[i]);
        }
    }

    /* Our response to the "SUBMIT" is to send "/ssi_cgi.shtml".
     * The extension ".shtml" tells the server to insert some values
     * which show the user what has been done in response.
     */
    return "/ssi_cgi.shtml";
}

/* cgi-handler triggered by a request for "/leds.cgi" */
const char *
cgi_handler_dial(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    int i=0;

    Serial.printf("cgi_handler_dial called with index %d\n", iIndex);

    // for (i = 0; i < iNumParams; i++){
    //     /* check if parameter is "led" */
    //     if (strcmp(pcParam[i] , "led") == 0){
    //         /* look ar argument to find which led to turn on */
    //         if(strcmp(pcValue[i], "1") == 0)
    //             Led_On(LED1);
    //         else if(strcmp(pcValue[i], "2") == 0)
    //             Led_On(LED2);
    //         else if(strcmp(pcValue[i], "3") == 0)
    //             Led_On(LED3);
    //         else if(strcmp(pcValue[i], "4") == 0)
    //             Led_On(LED4);
    //     }
    // }

    int slider_idx;
    float slider_value;

    for (i = 0; i < iNumParams; i++){
        // if (strcmp(pcParam[i] , "v") == 0){
        //     Serial.printf("v: %s", pcValue[i]);
        // }
        //Serial.printf("%s: %s", pcParam[i], pcValue[i]);
        // Process index parameter
        if (!strcmp(pcParam[i], "s")) {
            slider_idx = atoi(pcValue[i]);
        } else if (!strcmp(pcParam[i], "v")) {
            slider_value = atof(pcValue[i]);
        }
    }
    //Serial.printf("Slider %d: %f\n", slider_idx, slider_value);
    //serial_->sendFloatMessage(UART_Common::slider, slider_idx, slider_value);
    // Update internal state
    switch (slider_idx) {
        case 0: {  // expl range
            gAppState.exploration_range = slider_value;
        } break;
        case 1: {  // n iterations
            gAppState.n_iterations = slider_value;
        } break;
    }
    /* Our response to the "SUBMIT" is to simply send the same page again*/
    return "/blank.html";
}

const char *
cgi_handler_button(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    int i=0;

    Serial.printf("cgi_handler_button called with index %d\n", iIndex);

    // for (i = 0; i < iNumParams; i++){
    //     /* check if parameter is "led" */
    //     if (strcmp(pcParam[i] , "led") == 0){
    //         /* look ar argument to find which led to turn on */
    //         if(strcmp(pcValue[i], "1") == 0)
    //             Led_On(LED1);
    //         else if(strcmp(pcValue[i], "2") == 0)
    //             Led_On(LED2);
    //         else if(strcmp(pcValue[i], "3") == 0)
    //             Led_On(LED3);
    //         else if(strcmp(pcValue[i], "4") == 0)
    //             Led_On(LED4);
    //     }
    // }

    int button_idx;
    int button_value;

    for (i = 0; i < iNumParams; i++){
        // if (strcmp(pcParam[i] , "v") == 0){
        //     Serial.printf("v: %s", pcValue[i]);
        // }
        //Serial.printf("%s: %s", pcParam[i], pcValue[i]);
        // Process index parameter
        if (!strcmp(pcParam[i], "b")) {
            button_idx = atoi(pcValue[i]);
        } else if (!strcmp(pcParam[i], "v")) {
            button_value = atoi(pcValue[i]);
        }
    }
    //Serial.printf("Slider %d: %f\n", slider_idx, slider_value);
    //serial_->sendMessage(UART_Common::button, button_idx, button_value);
    // Update internal state
    switch (button_idx) {
        case toggle_explmode: {
            gAppState.current_expl_mode = static_cast<te_expl_mode>(button_value);
        } break;
        case toggle_dataset: {
            gAppState.current_dataset = button_value;
        } break;
        case toggle_model: {
            gAppState.current_model = button_value;
        } break;
        case toggle_training: {
            gAppState.current_nn_mode = static_cast<te_nn_mode>(button_value);
        } break;
    }
    /* Our response to the "SUBMIT" is to simply send the same page again*/
    return "/blank.html";
}


const char *
cgi_handler_poll(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    //int i=0;

    Serial.printf("cgi_handler_poll called with index %d\n", iIndex);

    /* Our response to the "SUBMIT" is to simply send the same page again*/
    return "{\"test\":19}";
}


/* initialize the CGI handler */
void
cgi_init()
{

    http_set_cgi_handlers(cgi_handlers, NUMCGIHANDLERS);

    // for(int i = LED1; i <= LED4; i++){
    //     gpio_init(i);
    //     gpio_set_dir(i, GPIO_OUT);
    //     gpio_put(i, 0);
    // }
}

/* led control and debugging info */
void
Led_On(int led)
{
    Serial.printf("GPIO%d on\n", led);
    //gpio_put(led, 1);
}

void
Led_Off(int led)
{
    Serial.printf("GPIO%d off\n", led);
    //gpio_put(led, 0);
}




