#ifndef _ARCH_SYSTEM_H
#define _ARCH_SYSTEM_H

#include <scandal/types.h>

#define     __I     volatile const          /*!< defines 'read only' permissions      */
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */

#define __DSB()                           __dsb(0)

#define SCS_BASE            (0xE000E000)                              /*!< System Control Space Base Address */
#define CoreDebug_BASE      (0xE000EDF0)                              /*!< Core Debug Base Address           */
#define SysTick_BASE        (SCS_BASE +  0x0010)                      /*!< SysTick Base Address              */
#define NVIC_BASE           (SCS_BASE +  0x0100)                      /*!< NVIC Base Address                 */
#define SCB_BASE            (SCS_BASE +  0x0D00)                      /*!< System Control Block Base Address */

#define SCB_AIRCR_VECTKEY_Pos              16                                             /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFul << SCB_AIRCR_VECTKEY_Pos)            /*!< SCB AIRCR: VECTKEY Mask */
#define SCB_AIRCR_SYSRESETREQ_Pos           2                                             /*!< SCB AIRCR: SYSRESETREQ Position */
#define SCB_AIRCR_SYSRESETREQ_Msk          (1ul << SCB_AIRCR_SYSRESETREQ_Pos)             /*!< SCB AIRCR: SYSRESETREQ Mask */

typedef struct {
  __I  uint32_t CPUID;                        /*!< Offset: 0x00  CPU ID Base Register                                  */
  __IO uint32_t ICSR;                         /*!< Offset: 0x04  Interrupt Control State Register                      */
       uint32_t RESERVED0;                                      
  __IO uint32_t AIRCR;                        /*!< Offset: 0x0C  Application Interrupt / Reset Control Register        */
  __IO uint32_t SCR;                          /*!< Offset: 0x10  System Control Register                               */
  __IO uint32_t CCR;                          /*!< Offset: 0x14  Configuration Control Register                        */
       uint32_t RESERVED1;                                      
  __IO uint32_t SHP[2];                       /*!< Offset: 0x1C  System Handlers Priority Registers. [0] is RESERVED   */
  __IO uint32_t SHCSR;                        /*!< Offset: 0x24  System Handler Control and State Register             */
       uint32_t RESERVED2[2];                                   
  __IO uint32_t DFSR;                         /*!< Offset: 0x30  Debug Fault Status Register                           */
} SCB_Type;                                                


#define SCB                 ((SCB_Type *)           SCB_BASE)         /*!< SCB configuration struct          */
#define SysTick             ((SysTick_Type *)       SysTick_BASE)     /*!< SysTick configuration struct      */
#define NVIC                ((NVIC_Type *)          NVIC_BASE)        /*!< NVIC configuration struct         */
#define CoreDebug           ((CoreDebug_Type *)     CoreDebug_BASE)   /*!< Core Debug configuration struct   */

#endif
