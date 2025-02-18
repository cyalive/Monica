/**
 * @file monica.cpp
 * @author Forairaaaaa
 * @brief 
 * @version 0.1
 * @date 2023-05-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <mooncake.h>
#include "user_apps/user_apps.h"
#include "hardware_manager/hardware_manager.h"
#include "user_apps/public_assets/assets.h"

#include <lv_examples.h>


static MOONCAKE::Mooncake mooncake_ui;
static HM::Hardware_Manager hardware_manager;





int btn_value = 0;

class AppTest : public MOONCAKE::APP_BASE {
    private:

        lv_obj_t* screen;
        int bbb1;
        int bbb2;

        uint32_t ticks;

        static void event_handler(lv_event_t * e)
        {
            lv_event_code_t code = lv_event_get_code(e);

            if(code == LV_EVENT_CLICKED) {

                btn_value = *(int*)lv_event_get_user_data(e);
                
            }

        }


    public:
        AppTest(const char* name, void* icon = nullptr) {
            setAppName(name);
            setAppIcon(icon);
        }

        void onSetup() {
        }

        /* Life cycle */
        void onCreate() {
            printf("[%s] onCreate\n", getAppName().c_str());

            setAllowBgRunning(false);

        }
        void onResume() {
            printf("[%s] onResume\n", getAppName().c_str());


            btn_value = 0;


            screen = lv_obj_create(NULL);
            lv_scr_load_anim(screen, LV_SCR_LOAD_ANIM_MOVE_TOP, 50, 0, false);
            

            lv_obj_t * label;

            lv_obj_t * btn1 = lv_btn_create(screen);
            bbb1 = 1;
            lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, (void*)&bbb1);
            lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);

            label = lv_label_create(btn1);
            // lv_obj_add_flag(btn1, LV_OBJ_FLAG_CHECKABLE);
            lv_obj_add_state(btn1, LV_STATE_CHECKED);
            lv_label_set_text(label, "Quit App");
            lv_obj_center(label);



            lv_obj_t * btn2 = lv_btn_create(screen);
            bbb2 = 2;
            lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, (void*)&bbb2);
            lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 40);

            label = lv_label_create(btn2);
            lv_label_set_text(label, "Run background");
            lv_obj_center(label);



        }
        void onRunning() {
            // printf("[%s] onRunning\n", getAppName().c_str());

            if ((lv_tick_get() - ticks) > 1000) {
                printf("[%s] onRunning\n", getAppName().c_str());
                ticks = lv_tick_get();
            }


            // printf("%d\n", btn_value);
            if (btn_value == 1) {
                setAllowBgRunning(false);
                destroyApp();
            }
            else if (btn_value == 2) {
                setAllowBgRunning(true);
                closeApp();
            }



        }   
        void onRunningBG() {
            // printf("[%s] onRunningBG\n", getAppName().c_str());

            if ((lv_tick_get() - ticks) > 1000) {
                printf("[%s] onRunningBG\n", getAppName().c_str());
                ticks = lv_tick_get();
            }
        }
        void onPause() {
            printf("[%s] onPause\n", getAppName().c_str());
        }
        void onDestroy() {
            printf("[%s] onDestroy\n", getAppName().c_str());
        }
};






extern "C" void app_main(void)
{

    
    /* Hardware init */
    hardware_manager.init();

    // Print SD card contents
    hardware_manager.sd.printDirectory("/sdcard");

    // Test display
    printf("Testing display with red color...\n");
    hardware_manager.disp.fillScreen(0xFF0000);  // 红色
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Create a simple LVGL test UI
    printf("Creating LVGL test UI...\n");
    
    // Create a screen
    lv_obj_t * scr = lv_obj_create(NULL);
    lv_scr_load(scr);
    
    // Create a button
    lv_obj_t * btn = lv_btn_create(scr);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
    
    // Create a label on the button
    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, "Hello World!");
    lv_obj_center(label);

    lv_demo_widgets();

    /* Original mooncake code - commented out
    // while (1) {
    //     printf("666\n");
    //     vTaskDelay(1000);
    // }

    // UI framwork init
    // mooncake_ui.setDisplay(hardware_manager.disp.width(), hardware_manager.disp.height());
    // mooncake_ui.init();
    // mooncake_ui.installBuiltinApps();

    // Set to same database
    // hardware_manager.setMooncake(&mooncake_ui);

    // Install Apps
    // MOONCAKE::APP_BASE* app_ptr = nullptr;
    // app_ptr = new AppTest("333", (void*)&ui_img_app_icon_hdpi_boxing_png);
    // mooncake_ui.install(app_ptr);
    // app_ptr = new AppTest("444", (void*)&ui_img_app_icon_hdpi_camera_png);
    // mooncake_ui.install(app_ptr);
    // app_ptr = new AppTest("111", (void*)&ui_img_app_icon_hdpi_badminton_png);
    // mooncake_ui.install(app_ptr);
    // app_ptr = new AppTest("222", (void*)&ui_img_app_icon_hdpi_birdhead_png);
    // mooncake_ui.install(app_ptr);
    // app_ptr = new AppTest("666", (void*)&ui_img_app_icon_hdpi_cheers_png);
    // mooncake_ui.install(app_ptr);
    // app_ptr = new AppTest("999", (void*)&ui_img_app_icon_hdpi_pingpong_png);
    // mooncake_ui.install(app_ptr);
    // app_ptr = new AppTest("1010", (void*)&ui_img_app_icon_hdpi_weather_png);
    // mooncake_ui.install(app_ptr);
    // app_ptr = new AppTest("777", (void*)&ui_img_app_icon_hdpi_location_png);
    // mooncake_ui.install(app_ptr);
    // app_ptr = new AppTest("888", (void*)&ui_img_app_icon_hdpi_music_png);
    // mooncake_ui.install(app_ptr);
    */

    while (1) {
        // printf("Main loop iteration\n");  
        lv_timer_handler();
        // hardware_manager.update();
        // mooncake_ui.update();
        vTaskDelay(pdMS_TO_TICKS(5));
    }

}
