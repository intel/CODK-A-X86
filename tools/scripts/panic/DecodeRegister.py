#!/usr/bin/python
"""
Script used to decode specific registers
"""

class DecodeRegister(object):
    """
    decode registers according to the architecture
    """

def decode_cortexm4_ipsr(value):

    stm32_INTERRUPT_TYPES = [
        ("Window WatchDog Interrupt"),
        ("PVD through EXTI Line detection Interrupt"),
        ("Tamper and TimeStamp interrupts through the EXTI line"),
        ("RTC Wakeup interrupt through the EXTI line"),
        ("FLASH global Interrupt"),
        ("RCC global Interrupt"),
        ("EXTI Line0 Interrupt"),
        ("EXTI Line1 Interrupt"),
        ("EXTI Line2 Interrupt"),
        ("EXTI Line3 Interrupt"),
        ("EXTI Line4 Interrupt"),
        ("DMA1 Stream 0 global Interrupt"),
        ("DMA1 Stream 1 global Interrupt"),
        ("DMA1 Stream 2 global Interrupt"),
        ("DMA1 Stream 3 global Interrupt"),
        ("DMA1 Stream 4 global Interrupt"),
        ("DMA1 Stream 5 global Interrupt"),
        ("DMA1 Stream 6 global Interrupt"),
        ("ADC1, ADC2 and ADC3 global Interrupts"),
        ("CAN1 TX Interrupt"),
        ("CAN1 RX0 Interrupt"),
        ("CAN1 RX1 Interrupt"),
        ("CAN1 SCE Interrupt"),
        ("External Line[9:5] Interrupts"),
        ("TIM1 Break interrupt and TIM9 global interrupt"),
        ("TIM1 Update Interrupt and TIM10 global interrupt"),
        ("TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt"),
        ("TIM1 Capture Compare Interrupt"),
        ("TIM2 global Interrupt"),
        ("TIM3 global Interrupt"),
        ("TIM4 global Interrupt"),
        ("I2C1 Event Interrupt"),
        ("I2C1 Error Interrupt"),
        ("I2C2 Event Interrupt"),
        ("I2C2 Error Interrupt"),
        ("SPI1 global Interrupt"),
        ("SPI2 global Interrupt"),
        ("USART1 global Interrupt"),
        ("USART2 global Interrupt"),
        ("USART3 global Interrupt"),
        ("External Line[15:10] Interrupts"),
        ("RTC Alarm (A and B) through EXTI Line Interrupt"),
        ("USB OTG FS Wakeup through EXTI line interrupt"),
        ("TIM8 Break Interrupt and TIM12 global interrupt"),
        ("TIM8 Update Interrupt and TIM13 global interrupt"),
        ("TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt"),
        ("TIM8 Capture Compare Interrupt"),
        ("DMA1 Stream7 Interrupt"),
        ("FMC global Interrupt"),
        ("SDIO global Interrupt"),
        ("TIM5 global Interrupt"),
        ("SPI3 global Interrupt"),
        ("UART4 global Interrupt"),
        ("UART5 global Interrupt"),
        ("TIM6 global and DAC1&2 underrun error  interrupts"),
        ("TIM7 global interrupt"),
        ("DMA2 Stream 0 global Interrupt"),
        ("DMA2 Stream 1 global Interrupt"),
        ("DMA2 Stream 2 global Interrupt"),
        ("DMA2 Stream 3 global Interrupt"),
        ("DMA2 Stream 4 global Interrupt"),
        ("Ethernet global Interrupt"),
        ("Ethernet Wakeup through EXTI line Interrupt"),
        ("CAN2 TX Interrupt"),
        ("CAN2 RX0 Interrupt"),
        ("CAN2 RX1 Interrupt"),
        ("CAN2 SCE Interrupt"),
        ("USB OTG FS global Interrupt"),
        ("DMA2 Stream 5 global interrupt"),
        ("DMA2 Stream 6 global interrupt"),
        ("DMA2 Stream 7 global interrupt"),
        ("USART6 global interrupt"),
        ("I2C3 event interrupt"),
        ("I2C3 error interrupt"),
        ("USB OTG HS End Point 1 Out global interrupt"),
        ("USB OTG HS End Point 1 In global interrupt"),
        ("USB OTG HS Wakeup through EXTI interrupt"),
        ("USB OTG HS global interrupt"),
        ("DCMI global interrupt"),
        ("Hash and RNG global interrupt"),
        ("FPU global interrupt"),
        ("UART7 global interrupt"),
        ("UART8 global interrupt"),
        ("SPI4 global Interrupt"),
        ("SPI5 global Interrupt"),
        ("SPI6 global Interrupt"),
        ("SAI1 global Interrupt"),
        ("LTDC global Interrupt"),
        ("LTDC Error global Interrupt"),
        ("DMA2D global Interrupt")]

    output_string = ""
    IPSR = value & 0x1FF
    IPSR_string = ("\tIPSR = %d " %(IPSR))
    if IPSR == 0:
        IPSR_string += ("(Thread mode, NoException)")
    elif IPSR == 1:
        IPSR_string += ("(Reset)")
    elif IPSR == 2:
        IPSR_string += ("(Non-Maskable Interrupt, System handlers)")
    elif IPSR == 3:
        IPSR_string += ("(HardFault, Fault handler)")
    elif IPSR == 11:
        IPSR_string += ("(SVcall, System handlers)")
    elif IPSR == 14:
        IPSR_string += ("(PendSV, System handlers)")
    elif IPSR == 15:
        IPSR_string += ("(SysTick, System handlers)")
    elif IPSR > 15 and IPSR < 91:
        IPSR_string += ("(IRQ%d: %s)" %((IPSR - 16), stm32_INTERRUPT_TYPES[IPSR - 17]))
    else:
        IPSR_string += ("(Reserved)")

    output_string += ("%s" %(IPSR_string))
    return output_string

def decode_nrf51_ipsr(value):

    nRF51_INTERRUPT_TYPES = [
        ("Power Clock"),
        ("Radio"),
        ("UART0"),
        ("SPI0 / TWI0"),
        ("SPI1 / TWI1"),
        ("GPIOTE"),
        ("ADC"),
        ("TIMER0"),
        ("TIMER1"),
        ("TIMER2"),
        ("RTC0"),
        ("TEMP"),
        ("RNG"),
        ("ECB"),
        ("CCM_AAR"),
        ("Watchdog"),
        ("RTC1"),
        ("QDEC"),
        ("LPCOM"),
        ("SWI0"),
        ("SWI1"),
        ("SWI2"),
        ("SWI3"),
        ("SWI4"),
        ("SWI5")]

    output_string = ""
    IPSR = value & 0x3F
    IPSR_string = ("\tIPSR = %d " %(IPSR))
    if IPSR == 0:
        IPSR_string += ("(Thread mode, NoException)")
    elif IPSR == 1:
        IPSR_string += ("(Reset)")
    elif IPSR == 2:
        IPSR_string += ("(Non-Maskable Interrupt, System handlers)")
    elif IPSR == 3:
        IPSR_string += ("(HardFault, Fault handler)")
    elif IPSR == 11:
        IPSR_string += ("(SVcall, System handlers)")
    elif IPSR == 14:
        IPSR_string += ("(PendSV, System handlers)")
    elif IPSR == 15:
        IPSR_string += ("(SysTick, System handlers)")
    elif IPSR > 15 and IPSR < 48:
        IPSR_string += ("(IRQ%d: %s)" %((IPSR - 16), nRF51_INTERRUPT_TYPES[IPSR - 17]))
    else:
        IPSR_string += ("(Reserved)")

    output_string += ("%s" %(IPSR_string))
    return output_string


def decode_nrf51_xpsr(value):
    output_string = ""
    if value.bit_length() >= 28:
        APSR = value >> 28
    else:
        APSR = 0

    if value.bit_length() >= 24:
        EPSR_string = ("\tEPSR = %d (Thumb state)" %((value >> 24) & 0x1))
    else:
        EPSR_string = ("\tEPSR = 0 (Thumb state)")

    APSR_string = ("\tAPSR = %d " %(APSR))

    if APSR == 1:
        APSR_string += ("(Overflow)")
    elif APSR == 2:
        APSR_string += ("(Carry)")
    elif APSR == 4:
        APSR_string += ("(Zero)")
    elif APSR == 8:
        APSR_string += ("(Negative)")
    else:
        APSR_string += ("(Reserved)")

    output_string += ("%s %s" %(APSR_string, EPSR_string))
    return output_string

def decode_cortexm4_xpsr(value):
    output_string = ""
    if value.bit_length() >= 27:
        APSR = value >> 27
    else:
        APSR = 0

    if value.bit_length() >= 24:
        EPSR_string = ("\tEPSR = %d (Thumb state)" %((value >> 24) & 0x1))
    else:
        EPSR_string = ("\tEPSR = 0 (Thumb state)")

    APSR_string = ("\tAPSR = %d " %(APSR))

    if APSR == 1:
        APSR_string += ("(DSP Overflow)")
    elif APSR == 2:
        APSR_string += ("(Overflow)")
    elif APSR == 4:
        APSR_string += ("(Carry)")
    elif APSR == 8:
        APSR_string += ("(Zero)")
    elif APSR == 16:
        APSR_string += ("(Negative)")
    else:
        APSR_string += ("(Reserved)")

    output_string += ("%s %s" %(APSR_string, EPSR_string))
    return output_string



