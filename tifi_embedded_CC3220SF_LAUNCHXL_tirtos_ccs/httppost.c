/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== httpget.c ========
 *  HTTP Client GET example application
 */

/* BSD support */
#include "string.h"
#include <ti/display/Display.h>
#include <ti/net/http/httpclient.h>
#include "semaphore.h"
#include <pthread.h>


/* TODO: move these when PIR GPIO is move */
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h"


#include <stdio.h>
#include <unistd.h>
#include "TMP116.h"

#define HOSTNAME            "http://www.ti-fi-uofsc.com"
#define REQUEST_URI_TMP     "/Williams/api/williams-tmp/post-data/"
#define REQUEST_URI_PIR     "/Williams/api/williams-motion/post-data/"
#define USER_AGENT          "HTTPClient (ARM; TI-RTOS)"
#define CONTENT_TYPE        "application/json; charset=utf-8"
#define HTTP_MIN_RECV       (256)
#define PAYLOAD_BUF_LEN     32
#define TEMP_PERIOD_S       5

#define THREADSTACKSIZE    4096

extern Display_Handle display;
extern sem_t ipEventSyncObj;
extern void printError(char *errString, int code);

void http_post_data(uint16_t data_val, char *payload_buf,
                    char *response_buf, uint32_t resp_buf_len,
                    const char *request_uri);

static void pir_callback_fxn(uint_least8_t index);
void* PIRThread(void* arg0);

static bool pir_flag;
sem_t semPIR;

/* TODO: maintain this better, rename / re-comment to correct terms */

/*
 *  ======== tempTask ========
 *  Makes a HTTP POST with a measured temperature every TEMP_PERIOD_S seconds
 */
void* tempTask(void *pvParameters)
{
    char data[HTTP_MIN_RECV];    /* TODO: rename this */
    /* TODO: name this better and use the JSON library */
    char payload[PAYLOAD_BUF_LEN];

    float temp = 0.0;
    int8_t temp_status;

    /* Initialize the GPIO for PIR sensor */
    /* TODO: move this to its own file */
    GPIO_init();
    /* GPIO_setConfig(CONFIG_GPIO_PIR, GPIO_CFG_PULL_NONE_INTERNAL | GPIO_CFG_IN_INT_RISING);*/
    GPIO_setCallback(CONFIG_GPIO_PIR, pir_callback_fxn);
    GPIO_enableInt(CONFIG_GPIO_PIR);

    /* Initialize the I2C bus */
    /* TODO: move this to a function in its own file, an I2C utility file */
    I2C_Handle i2c;
    I2C_Params i2c_params;
    I2C_init();

    /* TODO: move initialization to a different thread that attaches the tempTask thread and returns */
    pthread_t pir_thread;
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    /* Set up the semaphore and thread for the PIR interrupt */
    retc = sem_init(&semPIR, 0, 0);
    if (retc == -1) {
        while(1);
    }

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 1;
    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);
    if (retc != 0) {
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, THREADSTACKSIZE);
    if(retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&pir_thread, &pAttrs, PIRThread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while(1);
    }

    /* Initialize I2C for the temperature sensor */
    I2C_Params_init(&i2c_params);
    i2c_params.bitRate = I2C_400kHz;

    i2c = I2C_open(CONFIG_I2C, &i2c_params);
    if (i2c == NULL)
    {
        Display_printf(display, 0, 0, "Error Initializing I2C\n");
        while (1)
            ;
    }
    else
    {
        Display_printf(display, 0, 0, "I2C Initialized!\n");
    }
    /* End I2C initialization */

    Display_printf(display, 0, 0, "Sending a HTTP POST request to '%s'\n",
    HOSTNAME);

    while (1)
    {

        temp_status = get_temperature(i2c, &temp);
        if (temp_status != 0) {
            Display_printf(display, 0, 0, "Error reading temperature.");
            return 0;
        }

        http_post_data((uint16_t) temp, payload, data, sizeof(data), REQUEST_URI_TMP);
        sleep(TEMP_PERIOD_S);
    }

}

void http_post_data(uint16_t data_val, char *payload_buf,
                    char *response_buf, uint32_t resp_buf_len,
                    const char *request_uri)
{
    HTTPClient_Handle httpClientHandle;
    int16_t statusCode;
    int16_t len = 0;
    int16_t ret = 0;
    bool moreDataFlag = false;

    sprintf(payload_buf, "{\"data\":\"%d\"}", data_val);

    httpClientHandle = HTTPClient_create(&statusCode, 0);
    if (statusCode < 0)
    {
        printError("httpTask: creation of http client handle failed",
                   statusCode);
    }

    ret = HTTPClient_setHeader(httpClientHandle,
                               HTTPClient_HFIELD_REQ_USER_AGENT,
                               USER_AGENT, strlen(USER_AGENT),
                               HTTPClient_HFIELD_PERSISTENT);

    ret |= HTTPClient_setHeader(httpClientHandle,
                                HTTPClient_HFIELD_REQ_CONTENT_TYPE,
                                CONTENT_TYPE, strlen(CONTENT_TYPE),
                                HTTPClient_HFIELD_PERSISTENT);
    if (ret < 0)
    {
        printError("httpTask: setting request header failed", ret);
    }

    ret = HTTPClient_connect(httpClientHandle, HOSTNAME, 0, 0);
    if (ret < 0)
    {
        printError("httpTask: connect failed", ret);
    }

    ret = HTTPClient_sendRequest(httpClientHandle, HTTP_METHOD_POST,
                                 request_uri,
                                 payload_buf,
                                 strlen(payload_buf),
                                 0);

    Display_printf(display, 0, 0, "Payload Length: %d\n", strlen(payload_buf));
    Display_printf(display, 0, 0, "Payload: %s\n", payload_buf);

    if (ret < 0)
    {
        printError("httpTask: send failed", ret);
    }

    if (ret != HTTP_SC_CREATED)
    {
        printError("httpTask: cannot get status", ret);
    }

    Display_printf(display, 0, 0, "HTTP Response Status Code: %d\n", ret);

    len = 0;
    do
    {
        ret = HTTPClient_readResponseBody(httpClientHandle, response_buf,
                                          resp_buf_len, &moreDataFlag);
        if (ret < 0)
        {
            printError("httpTask: response body processing failed", ret);
        }
        Display_printf(display, 0, 0, "%.*s \r\n", ret, response_buf);
        len += ret;
    }
    while (moreDataFlag);

    Display_printf(display, 0, 0, "Received %d bytes of payload\n", len);

    ret = HTTPClient_disconnect(httpClientHandle);
    if (ret < 0)
    {
        printError("httpTask: disconnect failed", ret);
    }

    HTTPClient_destroy(httpClientHandle);
}

void pir_callback_fxn(uint_least8_t index)
{
    sem_post(&semPIR);
}

void* PIRThread(void* arg0) {
    char payload[PAYLOAD_BUF_LEN];
    char data[HTTP_MIN_RECV];

    while (1) {
        sem_wait(&semPIR);
        Display_printf(display, 0, 0, "Movement detected!\n");
        http_post_data(1, payload, data, sizeof(data), REQUEST_URI_PIR);
    }
}
