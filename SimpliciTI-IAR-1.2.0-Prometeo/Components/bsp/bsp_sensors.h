/**************************************************************************************************
  Revised:        $Date: 03-07-2015 $
  Revision:       $Revision: 1 $

  XXX-albarc      Archivo con los headers y constantes de bsp_sensors. Portado y modificado
                  partiendo del archivo medir.h de los proyectos originales de prometeo
                  con la HAL.
**************************************************************************************************/

#ifndef BSP_SENSORS_H
#define BSP_SENSORS_H

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "bsp_board_defs.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Defines
 * ------------------------------------------------------------------------------------------------
 */
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define SHT75_PORT_DIR     P2DIR
#define SHT75_PORT_SEL     P2SEL
#define SHT75_PORT_OUT     P2OUT
#define SHT75_PORT_IN      P2IN
#define SHT75_PORT_IE      P2IE
#define SHT75_PORT_IES     P2IES
#define SHT75_PORT_IFG     P2IFG
#define SHT75_SCK          BIT3
#define SHT75_SDA          BIT4

#define NUM_DIRECCIONES_VELETA    16
#define LIM_SECTOR_1          11.25
#define LIM_SECTOR_2          33.75
#define LIM_SECTOR_3          56.25
#define LIM_SECTOR_4          78.75
#define LIM_SECTOR_5          101.25
#define LIM_SECTOR_6          123.75
#define LIM_SECTOR_7          146.25
#define LIM_SECTOR_8          168.75
#define LIM_SECTOR_9          191.25
#define LIM_SECTOR_10         213.75
#define LIM_SECTOR_11         236.25
#define LIM_SECTOR_12         258.75
#define LIM_SECTOR_13         281.25
#define LIM_SECTOR_14         303.75
#define LIM_SECTOR_15         326.25
#define LIM_SECTOR_16         348.75
#define DIR_1                 0
#define DIR_2                 23
#define DIR_3                 45
#define DIR_4                 68
#define DIR_5                 90
#define DIR_6                 113
#define DIR_7                 135
#define DIR_8                 158
#define DIR_9                 180
#define DIR_10                203
#define DIR_11                225
#define DIR_12                248
#define DIR_13                270
#define DIR_14                293
#define DIR_15                315
#define DIR_16                338

/* ------------------------------------------------------------------------------------------------
 *                                        Prototypes
 * ------------------------------------------------------------------------------------------------
 */
void start_transmission_SHT75 (void);
void reset_connection_SHT75 (void);
void read_temperature_SHT75 (void);
void read_humidity_SHT75 (void);
void read_2bytes_SHT75 (uint8_t *field);
int SHT75_medirTemperatura (void);
uint16_t SHT75_medirHumedad (void);
static void SHT75_SDA_line_ISR (void);
static void anemometro_ISR (void);
void initAnemometro (void);
uint16_t calcularVelocidadAnemometro (uint32_t segundos);
uint16_t calcularDireccionVeleta (void);
void resetearRepeticionesVeleta (void);
uint16_t calcularDireccionMasRepetida (void);

/**********************************************************************************/
#endif