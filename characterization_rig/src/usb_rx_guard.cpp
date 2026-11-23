/* STM32duino 3.0.0's CDC queue-full path calls PrepareReceive(NULL, 0).
 * On the F103 USB peripheral that rearms RX VALID with a null destination:
 * the next packet faults in USB_ReadPMA. NAK until CDC_resume_receive()
 * supplies real buffer space instead. Keep this workaround local to the
 * firmware, not an untracked edit to the installed framework package.
 */
#if defined(ARDUINO_ARCH_STM32) && defined(STM32F103xB)
#include <stm32f1xx_hal.h>
#include <usbd_def.h>

extern "C" {
extern PCD_HandleTypeDef g_hpcd;
USBD_StatusTypeDef __real_USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev,
                                               uint8_t ep, uint8_t *buf,
                                               uint32_t size);
USBD_StatusTypeDef __wrap_USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev,
                                               uint8_t ep, uint8_t *buf,
                                               uint32_t size) {
    if (buf == nullptr && size == 0 && (ep & 0x7f) != 0) {
        PCD_SET_EP_RX_STATUS(g_hpcd.Instance, ep & 0x7f, USB_EP_RX_NAK);
        return USBD_OK;
    }
    return __real_USBD_LL_PrepareReceive(pdev, ep, buf, size);
}
}
#endif
