#include "freeRTOS_tasks.h"



TaskHandle_t  led_toggle_task_handle;         /**< Reference to LED0 toggling task. */
TaskHandle_t  ble_toggle_task_handle;         /**< Reference to BLE task. */
TaskHandle_t  control_toggle_task_handle;     /**< Reference to control task. */
TaskHandle_t  debug_toggle_task_handle;       /**< Reference to debug task. */
TaskHandle_t  imu_toggle_task_handle;         /**< Reference to imu task. */
TaskHandle_t  usbd_toggle_task_handle;        /**< Reference to usbd task. */
TaskHandle_t lora_toggle_task_handle;        /**< Reference to lora task. */


/**
 *  basic process to cycle start all desired freeRTOS tasks
 * @return  error state
 */
int32_t freeRTOS_StartTasks(void)
{
    int32_t error = NRF_SUCCESS; 


// TODO Remove this lazy copy paste --- store in strucutre 

    /* Create task for LED0 blinking with priority set to 2 */
    UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0_TASK", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task_handle));

    UNUSED_VARIABLE(xTaskCreate(ble_task_function, "BLE_TASK", configMINIMAL_STACK_SIZE + 200, NULL, 2, &ble_toggle_task_handle));

    UNUSED_VARIABLE(xTaskCreate(lora_task_function, "LORA_TASK", configMINIMAL_STACK_SIZE + 200, NULL, 2, &lora_toggle_task_handle));


  //  UNUSED_VARIABLE(xTaskCreate(control_task_function, "CONTROL_TASK", configMINIMAL_STACK_SIZE + 200, NULL, 2, &control_toggle_task_handle));

//    UNUSED_VARIABLE(xTaskCreate(debug_task_function, "DEBUG_TASK", configMINIMAL_STACK_SIZE + 200, NULL, 2, &debug_toggle_task_handle));

 //   UNUSED_VARIABLE(xTaskCreate(imu_task_function, "IMU_TASK", configMINIMAL_STACK_SIZE + 200, NULL, 2, &imu_toggle_task_handle));

   // UNUSED_VARIABLE(xTaskCreate(usbd_toggle_task_handle, "USBD_TASK", configMINIMAL_STACK_SIZE + 200, NULL, 2, &usbd_toggle_task_handle));

    return error; 
}