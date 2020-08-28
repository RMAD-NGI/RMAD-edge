/*
 * nandflash-W29N01HV.h
 *
 *  Created on: 17. juni 2020
 *      Author: KVIYME
 */

#ifndef NANDFLASH_W29N01HV_H_
#define NANDFLASH_W29N01HV_H_



#ifdef __cplusplus
extern "C" {
#endif

#define NAND_W29N01HV_SPARESIZE       64 /**< Spare area size of Winbond's W29N01HVDINA. */

/* "Standard" byte positions in the spare area. */
#define NAND_SPARE_BADBLOCK_POS    0 /**< Spare area position of bad-block marker. */
#define NAND_SPARE_ECC0_POS        6 /**< Spare area position of ECC byte 0 (LSB). */
#define NAND_SPARE_ECC1_POS        7 /**< Spare area position of ECC byte 1.       */
#define NAND_SPARE_ECC2_POS        8 /**< Spare area position of ECC byte 2 (MSB). */

/** @brief NANDFLASH status enumerator. */
typedef enum
{
  NANDFLASH_W29N01HV_STATUS_OK         = 0,        /**< No errors detected.                      */
  NANDFLASH_W29N01HV_INVALID_DEVICE    = -1,       /**< Invalid (unsupported) flash device.      */
  NANDFLASH_W29N01HV_INVALID_ADDRESS   = -2,       /**< Invalid nand flash address.              */
  NANDFLASH_W29N01HV_WRITE_ERROR       = -3,       /**< Nand flash write error, block is "bad".  */
  NANDFLASH_W29N01HV_ECC_ERROR         = -4,       /**< Illegal ECC value read from spare area.  */
  NANDFLASH_W29N01HV_ECC_UNCORRECTABLE = -5,       /**< Uncorrectable data error in page.        */
  NANDFLASH_W29N01HV_INVALID_SETUP     = -6,       /**< Invalid parameter to NANDFLASH_Init().   */
  NANDFLASH_W29N01HV_NOT_INITIALIZED   = -7,       /**< Nand module has not been initialized.    */
} NANDFLASH_W29N01HV_Status_TypeDef;

/** @brief NANDFLASH device information structure. */
typedef struct
{
  uint32_t baseAddress;                     /**< The device base address in cpu memory map.   */
  uint8_t  manufacturerCode;                /**< The device manufacturer code.                */
  uint8_t  deviceCode;                      /**< The device ID .                              */
  uint32_t deviceSize;                      /**< Total device size in bytes.                  */
  uint32_t pageSize;                        /**< Device page size in bytes.                   */
  uint32_t spareSize;                       /**< Device page spare size in bytes.             */
  uint32_t blockSize;                       /**< Device block size in bytes.                  */
  uint32_t ecc;                             /**< Result of ECC generation from last read/written page. */
  uint8_t  spare[ NAND_W29N01HV_SPARESIZE ];   /**< Spare area content from last read page or spare operation. */
  int      dmaCh;                           /**< The DMA channel used, -1 if DMA is not used. */
} NANDFLASH_W29N01HV_Info_TypeDef;

bool  NANDFLASH_W29N01HV_AddressValid(uint32_t addr);
int   NANDFLASH_W29N01HV_CopyPage(uint32_t dstAddr, uint32_t srcAddr);
NANDFLASH_W29N01HV_Info_TypeDef *
NANDFLASH_W29N01HV_DeviceInfo(void);
int   NANDFLASH_W29N01HV_EccCorrect(uint32_t generatedEcc, uint32_t readEcc, uint8_t *data);
int   NANDFLASH_W29N01HV_EraseBlock(uint32_t address);
int   NANDFLASH_W29N01HV_Init(int dmaCh);
int   NANDFLASH_W29N01HV_MarkBadBlock(uint32_t address);
int   NANDFLASH_W29N01HV_ReadPage(uint32_t address, uint8_t *buffer);
int   NANDFLASH_W29N01HV_ReadSpare(uint32_t address, uint8_t *buffer);
int   NANDFLASH_W29N01HV_WritePage(uint32_t address, uint8_t *buffer);

//void W29N01HV_reset(void);

#ifdef __cplusplus
}
#endif

/** @} (end group NandFlash) */
/** @} (end group Drivers) */

#endif /* NANDFLASH_W29N01HV_H_ */
