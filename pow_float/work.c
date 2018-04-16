#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <jansson.h>
#include <curl/curl.h>

#include "work.h"
#define BUFFER_SIZE  (256 * 1024)  /* 256 KB */

#define GET_WORK	"http://%s:%s/get-work"
#define SUBMIT_WORK		"http://%s:%s/submit-work"
#define URL_SIZE     256

/* Return the offset of the first newline in text or the length of
   text if there's no newline */
static int newline_offset(const char *text)
{
    const char *newline = strchr(text, '\n');
    if(!newline)
        return strlen(text);
    else
        return (int)(newline - text);
}

struct write_result
{
    char *data;
    int pos;
};

static size_t write_response(void *ptr, size_t size, size_t nmemb, void *stream)
{
    struct write_result *result = (struct write_result *)stream;

    if(result->pos + size * nmemb >= BUFFER_SIZE - 1)
    {
        fprintf(stderr, "error: too small buffer\n");
        return 0;
    }

    memcpy(result->data + result->pos, ptr, size * nmemb);
    result->pos += size * nmemb;

    return size * nmemb;
}

static char *get_work(const char *url)
{
    CURL *curl = NULL;
    CURLcode status;
    struct curl_slist *headers = NULL;
    char *data = NULL;
    long code;

    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    if(!curl)
        goto error;

    data = malloc(BUFFER_SIZE);
    if(!data)
        goto error;

    struct write_result write_result = {
        .data = data,
        .pos = 0
    };

    curl_easy_setopt(curl, CURLOPT_URL, url);

    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_response);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &write_result);
	curl_easy_setopt(curl, CURLOPT_POST, 1);

    status = curl_easy_perform(curl);
    if(status != 0)
    {
        fprintf(stderr, "error: unable to request data from %s\n", url);
        fprintf(stderr, "%s\n", curl_easy_strerror(status));
        goto error;
    }

    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
    if(code != 200)
    {
        fprintf(stderr, "error: server responded with code %ld\n", code);
        goto error;
    }

    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);
    curl_global_cleanup();

    /* zero-terminate the result */
    data[write_result.pos] = '\0';

    return data;

error:
    if(data)
        free(data);
    if(curl)
        curl_easy_cleanup(curl);
    if(headers)
        curl_slist_free_all(headers);
    curl_global_cleanup();
    return NULL;
}

int submit_work(const char *str, const char *url)
{
	CURL *curl = curl_easy_init();
	if (curl) {
		curl_easy_setopt(curl, CURLOPT_URL, url);
		curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, strlen(str));
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, str);
		curl_easy_perform(curl);
	}
    curl_easy_cleanup(curl);
    curl_global_cleanup();
	return 0;
}

int main(int argc, char *argv[])
{
    size_t i;
    char *text;
    char url[URL_SIZE];
    char s_url[URL_SIZE];

    json_t *work;
    json_error_t error;

    if(argc != 3)
    {
        fprintf(stderr, "usage: %s USER REPOSITORY\n\n", argv[0]);
        fprintf(stderr, "List commits at USER's REPOSITORY.\n\n");
        return 2;
    }

    snprintf(url, URL_SIZE, GET_WORK, argv[1], argv[2]);
    snprintf(s_url, URL_SIZE, SUBMIT_WORK, argv[1], argv[2]);

    text = get_work(url);
    if(!text)
        return 1;

	printf("--------return text:%s\n", text);
    work = json_loads(text, 0, &error);
    free(text);
	 if(!work)
    {
        fprintf(stderr, "error: on line %d: %s\n", error.line, error.text);
        return 1;
    }

	struct BlockHeader header = {};
	int iRet = json_unpack(work, "{s:{s:i,s:i,s:s,s:s,s:i,s:{s:s},s:s,s:s,s:i,s:i}}", "header",
							"version", &(header.Version),
							"height", &(header.Height),
							"previous_block_hash", &(header.PreviousBlockId),
							"seed", &(header.Seed),
							"timestamp", &(header.Timestamp),
							"transaction_status", "bitmap", &(header.Status.Bitmap),
							"transaction_merkle_root", &(header.TransactionsRoot),
							"asset_merkle_root", &(header.AssetsRoot),
							"nonce", &(header.Nonce),
							"bits", &(header.Bits));
	assert( iRet == 0);
	printf("----------hedear's height:%ld\n", header.Height);
	// TO DO:  需要对header 计算nonce 轮询和人工智能挖矿算法处理。
    // for {
    //      nonce 0 .. &&


    //         AIHash(Hash(header), ) << header.Bits 
    //         if ---{ break}
    // }
    // header.Nonce = nonce;

	// submit work:
	char s_work[2048] = {};
	sprintf(s_work,
	  "{\"version\":%ld, \"height\":%ld, \"previous_block_hash\":%s, \"seed\":%s, \"timestamp\":%ld, \"transaction_status\":{\"bitmap\":%s}, \"transaction_merkle_root\":%s, \"asset_merkle_root\":%s, \"nonce\":%ld, \"bits\":%ld}\n",
	   header.Version, header.Height, header.PreviousBlockId, header.Seed, header.Timestamp,
	   header.Status.Bitmap, header.TransactionsRoot, header.AssetsRoot, header.Nonce, header.Bits);
	printf("----s_wrork:%s\n", s_work);
	submit_work(s_work, s_url);
    json_decref(work);
    return 0;
}
