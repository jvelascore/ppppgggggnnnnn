/**************************************************************************************************
  Revised:        $Date: 05-02-2015 $
  Revision:       $Revision: 1 $

  XXX-albarc      Las ISR de GPIO tienen que estar definidas en mrfi_board.c
                  porque pueden ser GDO0 y GDO2
**************************************************************************************************/

/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Generic button driver code file.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

/* ------------------------------------------------------------------------------------------------
 *                                            Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "bsp_external/mrfi_board_defs.h"
#include "bsp_digio.h"

BSP_ISR_FUNC_PTR port1_isr_tbl[8] = {0};
BSP_ISR_FUNC_PTR port2_isr_tbl[8] = {0};

/* ------------------------------------------------------------------------------------------------
 *                                  Global variables
 *  En este archivo deben ponerse todas las configuraciones de entrada/salida 
 *  digital que estén presentes en la placa (aparte de botones y leds)
 * ------------------------------------------------------------------------------------------------
 */
const digioConfig pinCono = {PIN_CONO_PORT, PIN_CONO_PIN, PIN_CONO_BITMASK, PIN_CONO_DIR,  PIN_CONO_INIT_VAL};


/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 *                        (comprobar que ningún pin corresponde a un GDO)
 **************************************************************************************************
 */
#if ((PIN_CONO_PORT == __mrfi_GDO0_PORT__) && (PIN_CONO_PIN == __mrfi_GDO0_BIT__))
  #error "ERROR:  El pin GPIO que estás intentando configurar se usa para GDO0."
#endif

#if ((PIN_CONO_PORT == __mrfi_GDO2_PORT__) && (PIN_CONO_PIN == __mrfi_GDO2_BIT__))
  #error "ERROR:  El pin GPIO que estás intentando configurar se usa para GDO2."
#endif


/**************************************************************************************************
 * @fn          BSP_InitDigio
 *
 * @brief       Initialize digital in/out.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void BSP_InitDigio(void)
{
  /* configure digio */
  BSP_ConfigDigio(&pinCono);
  
}

/**************************************************************************************************
 * @fn          BSP_ConfigDigio(const digioConfig* p)
 *
 * @brief       Configura un pin como entrada o salida digital.
 *
 * @param       digioConfig
 *
 * @return      none
 **************************************************************************************************
 */
uint8_t BSP_ConfigDigio(const digioConfig* p)
{
    register volatile uint8_t* dir;
    register volatile uint8_t* out;
    register const uint8_t bitmask = p->pin_bm;

    // Sanity check
    if ((bitmask == 0) || (bitmask != (uint8_t)BV(p->pin)))
    {
        return(BSP_DIGIO_ERROR);
    }

    switch(p->port)
    {
        case 1: P1SEL &= ~bitmask; out = &P1OUT; dir = &P1DIR; break;
        case 2: P2SEL &= ~bitmask; out = &P2OUT; dir = &P2DIR; break;
        case 3: P3SEL &= ~bitmask; out = &P3OUT; dir = &P3DIR; break;
        case 4: P4SEL &= ~bitmask; out = &P4OUT; dir = &P4DIR; break;
        default: return(BSP_DIGIO_ERROR);
    }

    if (p->dir == BSP_DIGIO_OUTPUT)
    {
        if (p->initval == 1)
        {
            *out |= bitmask;
        }
        else
        {
            *out &= ~bitmask;
        }
        *dir |= bitmask;
    }
    else // input
    {
        *out &= ~bitmask;
        *dir &= ~bitmask;
    }
    return(BSP_DIGIO_OK);
}

/**************************************************************************************************
 * @fn          BSP_DigioSet(const digioConfig* p)
 *
 * @brief       Pone un pin de salida a uno
 *
 * @param       digioConfig
 *
 * @return      none
 **************************************************************************************************
 */
uint8_t BSP_DigioSet(const digioConfig* p)
{
    if (p->dir == BSP_DIGIO_OUTPUT)
    {
        switch (p->port)
        {
            case 1: P1OUT |= p->pin_bm; break;
            case 2: P2OUT |= p->pin_bm; break;
            case 3: P3OUT |= p->pin_bm; break;
            case 4: P4OUT |= p->pin_bm; break;
            default: return(BSP_DIGIO_ERROR);
        }
        return(BSP_DIGIO_OK);
    }
    return(BSP_DIGIO_ERROR);
}

/**************************************************************************************************
 * @fn          BSP_DigioClear(const digioConfig* p)
 *
 * @brief       Pone un pin de salida a cero
 *
 * @param       digioConfig
 *
 * @return      none
 **************************************************************************************************
 */
uint8_t BSP_DigioClear(const digioConfig* p)
{
    if (p->dir == BSP_DIGIO_OUTPUT)
    {
        switch (p->port)
        {
            case 1: P1OUT &= ~p->pin_bm; break;
            case 2: P2OUT &= ~p->pin_bm; break;
            case 3: P3OUT &= ~p->pin_bm; break;
            case 4: P4OUT &= ~p->pin_bm; break;
            default: return(BSP_DIGIO_ERROR);
        }
        return(BSP_DIGIO_OK);
    }
    return(BSP_DIGIO_ERROR);
}

/**************************************************************************************************
 * @fn          BSP_DigioToggle(const digioConfig* p)
 *
 * @brief       Cambia el estado de un pin de salida
 *
 * @param       digioConfig
 *
 * @return      none
 **************************************************************************************************
 */
uint8_t BSP_DigioToggle(const digioConfig* p)
{
    if (p->dir == BSP_DIGIO_OUTPUT)
    {
        switch (p->port)
        {
            case 1: P1OUT ^= p->pin_bm; break;
            case 2: P2OUT ^= p->pin_bm; break;
            case 3: P3OUT ^= p->pin_bm; break;
            case 4: P4OUT ^= p->pin_bm; break;
            default: return(BSP_DIGIO_ERROR);
        }
        return(BSP_DIGIO_OK);
    }
    return(BSP_DIGIO_ERROR);
}

/**************************************************************************************************
 * @fn          BSP_DigioGet(const digioConfig* p)
 *
 * @brief       Lee un pin de entrada
 *
 * @param       digioConfig
 *
 * @return      none
 **************************************************************************************************
 */
uint8_t BSP_DigioGet(const digioConfig* p)
{
    if (p->dir == BSP_DIGIO_INPUT)
    {
        switch (p->port)
        {
            case 1: return (P1IN & p->pin_bm ? 1 : 0);
            case 2: return (P2IN & p->pin_bm ? 1 : 0);
            case 3: return (P3IN & p->pin_bm ? 1 : 0);
            case 4: return (P4IN & p->pin_bm ? 1 : 0);
            default: break;
        }
    }
    return(BSP_DIGIO_ERROR);
}

/**************************************************************************************************
 * @fn          BSP_DigioIntConnect(const digioConfig* p, BSP_ISR_FUNC_PTR func)
 *
 * @brief       Pone una función como ISR de la interrupción externa de un pin
 *
 * @param       digioConfig, puntero a función
 *
 * @return      none
 **************************************************************************************************
 */
uint8_t BSP_DigioIntConnect(const digioConfig *p, BSP_ISR_FUNC_PTR func)
{
    bspIState_t intState;
    BSP_ENTER_CRITICAL_SECTION(intState);
    switch (p->port)
    {
        case 1: port1_isr_tbl[p->pin] = func; break;
        case 2: port2_isr_tbl[p->pin] = func; break;
        default: BSP_EXIT_CRITICAL_SECTION(intState); return(BSP_DIGIO_ERROR);
    }
    BSP_DigioIntClear(p);
    BSP_EXIT_CRITICAL_SECTION(intState);
    return(BSP_DIGIO_OK);
}


/**************************************************************************************************
 * @fn          BSP_DigioIntEnable(const digioConfig* p)
 *
 * @brief       Habilita la interrupción externa de un pin
 *
 * @param       digioConfig
 *
 * @return      none
 **************************************************************************************************
 */
uint8_t BSP_DigioIntEnable(const digioConfig *p)
{
    switch (p->port)
    {
        case 1: P1IE |= p->pin_bm; break;
        case 2: P2IE |= p->pin_bm; break;
        default: return(BSP_DIGIO_ERROR);
    }
    return(BSP_DIGIO_OK);
}

/**************************************************************************************************
 * @fn          BSP_DigioIntDisable(const digioConfig* p)
 *
 * @brief       Deshabilita la interrupción externa de un pin
 *
 * @param       digioConfig
 *
 * @return      none
 **************************************************************************************************
 */
uint8_t BSP_DigioIntDisable(const digioConfig *p)
{
    switch (p->port)
    {
        case 1: P1IE &= ~p->pin_bm; break;
        case 2: P2IE &= ~p->pin_bm; break;
        default: return(BSP_DIGIO_ERROR);
    }
    return(BSP_DIGIO_OK);
}

/**************************************************************************************************
 * @fn          BSP_DigioIntClear(const digioConfig* p)
 *
 * @brief       Resetea el flag de la interrupción externa de un pin
 *
 * @param       digioConfig
 *
 * @return      none
 **************************************************************************************************
 */
uint8_t BSP_DigioIntClear(const digioConfig *p)
{
    switch (p->port)
    {
        case 1: P1IFG &= ~p->pin_bm; break;
        case 2: P2IFG &= ~p->pin_bm; break;
        default: return(BSP_DIGIO_ERROR);
    }
    return(BSP_DIGIO_OK);
}

/**************************************************************************************************
 * @fn          BSP_DigioIntSetEdge(const digioConfig* p,  uint8 edge)
 *
 * @brief       Establece el flanco de la interrupción externa de un pin
 *
 * @param       digioConfig, edge
 *
 * @return      none
 **************************************************************************************************
 */
uint8_t BSP_DigioIntSetEdge(const digioConfig *p, uint8_t edge)
{
    switch(edge)
    {
        case BSP_DIGIO_INT_FALLING_EDGE:
            switch(p->port)
            {
                case 1: P1IES |= p->pin_bm; break;
                case 2: P2IES |= p->pin_bm; break;
                default: return(BSP_DIGIO_ERROR);
            }
            break;

         case BSP_DIGIO_INT_RISING_EDGE:
            switch(p->port)
            {
                case 1: P1IES &= ~p->pin_bm; break;
                case 2: P2IES &= ~p->pin_bm; break;
                default: return(BSP_DIGIO_ERROR);
            }
            break;

         default:
            return(BSP_DIGIO_ERROR);
    }
    return(BSP_DIGIO_OK);
}
  
/**************************************************************************************************
*/