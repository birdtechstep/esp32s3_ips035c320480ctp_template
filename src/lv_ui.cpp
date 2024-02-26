/* ************************************************************************* *
 * @file lv_ui.cpp
 * ************************************************************************* */
/*********************
 *      INCLUDES
 *********************/
//#include <board_config.h>
#include <lv_ui.h>
#include <lvgl.h>
/*********************
 *      DEFINES
 *********************/
#define btn_color           lv_color_hex(0x2196F3)
#define label_border_color  lv_color_hex(0x607D8B)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void event_handler_cb_main_obj1(lv_event_t *e);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
lv_disp_t *disp;
lv_theme_t *theme;

//lv_obj_t * win_obj1;
  lv_obj_t * img_logo;
  lv_obj_t * label_obj0;

// main screen display
/* Color palette
LV_PALETTE_RED
LV_PALETTE_PINK
LV_PALETTE_PURPLE
LV_PALETTE_DEEP_PURPLE
LV_PALETTE_INDIGO
LV_PALETTE_BLUE
LV_PALETTE_LIGHT_BLUE
LV_PALETTE_CYAN
LV_PALETTE_TEAL
LV_PALETTE_GREEN
LV_PALETTE_LIGHT_GREEN
LV_PALETTE_LIME
LV_PALETTE_YELLOW
LV_PALETTE_AMBER
LV_PALETTE_ORANGE
LV_PALETTE_DEEP_ORANGE
LV_PALETTE_BROWN
LV_PALETTE_BLUE_GREY
LV_PALETTE_GREY
*/
void lv_things_widgets(void) {
    disp = lv_disp_get_default();
    theme = lv_theme_default_init(disp,                                    /*Use the DPI, size, etc from this display*/
                                  lv_palette_main(LV_PALETTE_BLUE),        /*Primary palette*/
                                  lv_palette_main(LV_PALETTE_BLUE_GREY),   /*Secondary palette*/
                                  false,                                   /*Light or dark mode*/
                                  LV_FONT_DEFAULT);                        /*Small, normal, large fonts*/
    lv_disp_set_theme(disp, theme);                                        /*Assign the theme to the display*/
    /*Change the active screen's background color*/

//    win_obj1 = lv_obj_create(lv_scr_act());   /*Create a parent object on the current screen*/
//    lv_obj_set_pos(win_obj1, 0, 0);	                     /*Set the position of the new object*/
//    lv_obj_set_size(win_obj1, 480, 320);	                 /*Set the size of the parent*/
//    lv_obj_clear_flag(win_obj1, LV_OBJ_FLAG_SCROLLABLE);
//    lv_obj_set_style_pad_all(win_obj1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    {
        //img_logo = lv_img_create(win_obj1);
        img_logo = lv_img_create(lv_scr_act());
        if(LV_THEME_DEFAULT_DARK) {
        lv_img_set_src(img_logo, "S:/assets/thinks_analytic_logo_white_300.bin"); // 300*128
        } else {
        lv_img_set_src(img_logo, "S:/assets/thinks_analytic_logo_blue_300.bin"); // 300*128
        }
        lv_obj_align(img_logo, LV_ALIGN_CENTER, 0, -10 );

        //label_obj0 = lv_label_create(win_obj1);
        label_obj0 = lv_label_create(lv_scr_act());
        lv_obj_set_style_text_font(label_obj0, &lv_font_montserrat_20, 0);
        lv_label_set_text(label_obj0, "-: DATA LOGGER :-");
        lv_obj_align(label_obj0, LV_ALIGN_BOTTOM_MID, 0, -10);        
    }

}

static void event_handler_cb_main_obj1(lv_event_t *e) {
    lv_event_code_t event = lv_event_get_code(e);
    if (event == LV_EVENT_PRESSED) {
        LV_LOG_USER("Button %d clicked", (int)lv_obj_get_index(obj));
        //Code here
    }
}
