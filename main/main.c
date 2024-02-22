#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "jsmn.h"
#include "si5351.h"
#include "blink_led.h"
#include "tinyusb_msc_storage.h"

typedef struct 
{
    uint32_t clk0;
    uint32_t clk2;
} si5351_clk_t;

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) 
{
    if(tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}

static int jsonti(jsmntok_t* tokens, int i, const char* json_string)
{
    char value_string[16];
    sprintf(value_string, "%.*s", tokens[i + 1].end - tokens[i + 1].start, 
                          json_string + tokens[i + 1].start);
    return atoi(value_string);
}

static void check_cfg_file(const char* fname)
{
    if (access(fname, F_OK) != 0) 
    {
        FILE *file = fopen(fname, "w");
        const char* text = "{\"clk0\": \"28000000\", \"clk2\": \"144000000\"}\n";
        fprintf(file, "%s", text);
        fclose(file);
    }
}

static bool parse_si5351_clk(const char* json_string, si5351_clk_t* si5351_clk)
{
    jsmn_parser jsmn;
    jsmn_init(&jsmn);

    jsmntok_t tokens[128];
    int tok_size = jsmn_parse(&jsmn, json_string, strlen(json_string), tokens, 128);

    if (tok_size < 1 || tokens[0].type != JSMN_OBJECT) return false;
    
    for (int i = 1; i < tok_size; i++)
    {
        if(jsoneq(json_string, &tokens[i], "clk0") == 0)
            si5351_clk->clk0 = jsonti(tokens, i, json_string);
        else if(jsoneq(json_string, &tokens[i], "clk2") == 0)
            si5351_clk->clk2 = jsonti(tokens, i, json_string);
    }

    return si5351_clk->clk0 && si5351_clk->clk2;
}

static bool get_si5351_clk(si5351_clk_t* si5351_clk)
{
    bool result = false;
    memset(si5351_clk, 0, sizeof(si5351_clk_t));

    const char* fname = "/data/settings.json";
    check_cfg_file(fname);

    FILE *file = fopen(fname, "r");
    if(!file) return false;

    fseek(file, 0, SEEK_END);
    long filelen = ftell(file);

    char* json_string = (char *)malloc((filelen + 1) * sizeof(char));
    memset(json_string, 0, filelen + 1);

    fseek(file, 0L, SEEK_SET);
    fread(json_string, sizeof(char), filelen, file);
    fclose(file);

    result = parse_si5351_clk(json_string, si5351_clk);

    free(json_string);
    return result;
}

static void access_file_system()
{
    si5351_clk_t clks;
    if(get_si5351_clk(&clks))
    {
        si5351_init(0);
        si5351_setupClk0(clks.clk0, SI5351_DRIVE_STRENGTH_4MA);
        si5351_setupClk2(clks.clk2, SI5351_DRIVE_STRENGTH_4MA);
        si5351_enableOutputs((1<<0) | (1<<2));
    }
}

void app_main(void)
{   
    si5351_init(0);
    si5351_setupClk0(28000000, SI5351_DRIVE_STRENGTH_4MA);
    si5351_setupClk2(144000000, SI5351_DRIVE_STRENGTH_4MA);
    si5351_enableOutputs((1<<0) | (1<<2));  
}
