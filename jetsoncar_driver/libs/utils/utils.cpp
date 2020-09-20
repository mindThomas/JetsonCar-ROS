/* Copyright (C) 2018-2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include "utils.hpp"
#include <curl/curl.h>

static size_t
WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    size_t realsize = size * nmemb;
    MemoryStruct *mem = (MemoryStruct *)userp;

    char *ptr = (char *)realloc(mem->memory, mem->size + realsize + 1);
    if(ptr == NULL) {
        /* out of memory! */
        printf("not enough memory (realloc returned NULL)\n");
        return 0;
    }

    mem->memory = ptr;
    memcpy(&(mem->memory[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->memory[mem->size] = 0;

    return realsize;
}

int CURL_Download(std::string URL, std::string filePath)
{
    CURLcode ret;
    CURL *curl;
    FILE *fptr;

    MemoryStruct chunk;
    chunk.memory = (char *)malloc(1);  /* will be grown as needed by the realloc above */
    chunk.size = 0;    /* no data at this point */

    fptr = fopen(filePath.c_str(), "wb");
    if (!fptr) return -1;

    curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, URL.c_str());
    curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "curl/7.35.0");
    curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 50L);
    curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, 1L);

    curl_easy_setopt(curl, CURLOPT_HEADER, 1L);  //Enable Headers
    //curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeDataOnStream);
    //curl_easy_setopt(curl, CURLOPT_WRITEDATA, stdout);   //Print data in STDOUT

    /* send all data to this function  */
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);

    /* we pass our 'chunk' struct to the callback function */
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);

    /* Here is a list of options the curl code used that cannot get generated
       as source easily. You may select to either not use them or implement
       them yourself.

       CURLOPT_WRITEDATA set to a objectpointer
       CURLOPT_WRITEFUNCTION set to a functionpointer
       CURLOPT_READDATA set to a objectpointer
       CURLOPT_READFUNCTION set to a functionpointer
       CURLOPT_SEEKDATA set to a objectpointer
       CURLOPT_SEEKFUNCTION set to a functionpointer
       CURLOPT_ERRORBUFFER set to a objectpointer
       CURLOPT_STDERR set to a objectpointer
       CURLOPT_HEADERFUNCTION set to a functionpointer
       CURLOPT_HEADERDATA set to a objectpointer

     */

    ret = curl_easy_perform(curl);

    curl_easy_cleanup(curl);
    curl = NULL;

    const char pattern[4] = {0x0D, 0x0A, 0x0D, 0x0A};
    size_t offset = strstr(chunk.memory, pattern) - chunk.memory + 4;

    // Write content to file
    fwrite(&chunk.memory[offset], 1, chunk.size - offset, fptr);
    fclose(fptr);

    free(chunk.memory);

    return (int) ret;
}