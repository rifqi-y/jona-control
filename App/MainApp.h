/**
 ******************************************************************************
 * @file    MainApp.h
 * @brief   Entry points for application logic (called from main.c).
 ******************************************************************************
 */

#ifndef MAINAPP_H_
#define MAINAPP_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Dipanggil sekali dari main() setelah HAL init.
void MainApp_Setup(void);

// Dipanggil terus-menerus dari while(1) di main().
void MainApp_Loop(void);

// Dipanggil oleh CDC_Receive_FS di usbd_cdc_if.c saat ada data USB masuk.
void onUsbDataReceived(uint8_t* Buf, uint32_t* Len);

#ifdef __cplusplus
}
#endif

#endif /* MAINAPP_H_ */