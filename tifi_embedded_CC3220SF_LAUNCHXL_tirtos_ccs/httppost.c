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

#include <stdio.h>
#include <unistd.h>
#include "TMP116.h"

#define HOSTNAME            "http://www.ti-fi-uofsc.com"
#define REQUEST_URI         "/Williams/api/williams-tmp/post-data/"
#define USER_AGENT          "HTTPClient (ARM; TI-RTOS)"
#define CONTENT_TYPE        "application/json; charset=utf-8"
#define HTTP_MIN_RECV       (256)

extern Display_Handle display;
extern sem_t ipEventSyncObj;
extern void printError(char *errString, int code);

static void http_post_data(uint16_t data_val, char *payload_buf,
                           char *response_buf, uint32_t resp_buf_len);

/* TODO: maintain this better, rename / re-comment to correct terms */

/*
 *  ======== httpTask ========
 *  Makes a HTTP GET request
 */
void* tempTask(void *pvParameters)
{
    char data[HTTP_MIN_RECV];    /* TODO: rename this */
    /* TODO: name this better and use the JSON library */
    char payload[32];

    float temp = 0.0;
    int8_t temp_status;

    /* Initialize the I2C bus */
    /* TODO: move this to a function in its own file, an I2C utility file */
    I2C_Handle i2c;
    I2C_Params i2c_params;
    I2C_init();

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

        http_post_data((uint16_t) temp, payload, data, sizeof(data));
        sleep(5);
    }

}

void http_post_data(uint16_t data_val, char *payload_buf,
                    char *response_buf, uint32_t resp_buf_len)
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
                                 REQUEST_URI,
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
