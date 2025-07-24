//0724 15:00-updated 

#include "ti_msp_dl_config.h"
#include "tsp_gpio.h"
#include "TSP_Encoder.h"
#include "mpu6050.h"


volatile int16_t Encoder_Count;


void tsp_Encoder_init(void)
{
    NVIC_EnableIRQ(PORTA_INT_IRQN);
}

int16_t tsp_Encoder_Get(void)
{
	int16_t Temp;
	Temp = Encoder_Count;
	Encoder_Count = 0;
	return Temp;
}

void GROUP1_IRQHandler(void)
{
    uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(
        PORTA_PORT,PORTA_PHA0_PIN|PORTA_PHB0_PIN);

    /*
     * Bitwise AND the pending interrupt with the pin you want to check,
     * then check if it is equal to the pins. Clear the interrupt status.
     */

    if ((gpioA & PORTA_PHB0_PIN) == PORTA_PHB0_PIN) 
    {
        if(PHB0() == 0)
        {
            if (PHB0() == 0)
            {
                if (PHA0() == 0)
                {
                    Encoder_Count ++;
                }
                else
                    Encoder_Count --;

		    }


        }

        DL_GPIO_clearInterruptStatus(PORTA_PORT, PORTA_PHB0_PIN);
    }

    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {    //
        case PORTC_INT_IIDX:            //
            switch (DL_GPIO_getPendingInterrupt(GPIOC))           //
            {                                           //
                case PORTC_IMU_INT_IIDX:           //
                    Read_Quad();        //
                    break;            //
            
            }
    }


    // if ((gpioA & PORTA_PHA0_PIN) == PORTA_PHA0_PIN) 
    // {
    //     if(PHA0() == 0)
    //     {
    //         if (PHA0() == 0)
    //         {
    //             if (PHB0() == 0)
    //             {
    //                 Encoder_Count --;
    //             }
    //             else
    //             {
    //                 Encoder_Count ++;
    //             }
    //         }

    //     }

    //     DL_GPIO_clearInterruptStatus(PORTA_PORT, PORTA_PHA0_PIN);
    // }




}


/*------------------------Test for Encoder------------------------------*/

// int16_t Num;

// int main(void)
// {
//     uint8_t value_ptr[6];
//     SYSCFG_DL_init();
//     tsp_tft18_init();
// 	   tsp_Encoder_init();
//     tsp_tft18_clear(BLACK);

//     while (1)
//     {
//         Num += tsp_Encoder_Get();
//         sprintf(value_ptr, "%05d", Num);
//         tsp_tft18_show_str(16, 0, value_ptr);
//         delay_1ms(50);
    
//     }
// }

/*------------------------Test for Encoder------------------------------*/
