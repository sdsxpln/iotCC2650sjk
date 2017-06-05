/******************************************************************************
 * Filename:       sjk_service.c
 *
 * Description:    This file contains the implementation of the service.
 *
 *                 Generated by:
 *                 BDS version: 1.0.2093.0
 *                 Plugin:      Texas Instruments CC26xx BLE SDK v2.1 GATT Server plugin 1.0.5 beta
 *                 Time:        Tue Jan 26 2016 22:57:47 GMT+01:00
 *

 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include <string.h>

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "sjk_service.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
* GLOBAL VARIABLES
*/

// Sjk_Service Service UUID
CONST uint8_t SjkServiceUUID[ATT_UUID_SIZE] =
{
  SJK_SERVICE_SERV_UUID_BASE128(SJK_SERVICE_SERV_UUID)
};

// SJK0 UUID
CONST uint8_t bs_SJK0UUID[ATT_UUID_SIZE] =
{
  BS_SJK0_UUID_BASE128(BS_SJK0_UUID)
};

// SJK1 UUID
CONST uint8_t bs_SJK1UUID[ATT_UUID_SIZE] =
{
  BS_SJK1_UUID_BASE128(BS_SJK1_UUID)
};


/*********************************************************************
 * LOCAL VARIABLES
 */

static SjkServiceCBs_t *pAppCBs = NULL;
static uint8_t bs_icall_rsp_task_id = INVALID_TASK_ID;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t SjkServiceDecl = { ATT_UUID_SIZE, SjkServiceUUID };
// Characteristic "SJK0" Properties (for declaration)
static uint8_t bs_SJK0Props = GATT_PROP_NOTIFY | GATT_PROP_READ;
// Characteristic "SJK0" Value variable
static uint8_t bs_SJK0Val[BS_SJK0_LEN] = {0};
// Length of data in characteristic "SJK0" Value variable, initialized to minimal size.
static uint16_t bs_SJK0ValLen = BS_SJK0_LEN_MIN;
// Characteristic "SJK0" Client Characteristic Configuration Descriptor
static gattCharCfg_t *bs_SJK0Config;

// DATA S
// Characteristic "SJK1" Properties (for declaration)
static uint8_t bs_SJK1Props = GATT_PROP_WRITE | GATT_PROP_READ;
// Characteristic "SJK1" Value variable
//sgkim
uint8_t bs_SJK1Val[BS_SJK1_LEN] = {0,};
// Length of data in characteristic "SJK1" Value variable, initialized to minimal size.
uint16_t bs_SJK1ValLen = BS_SJK1_LEN_MIN;

// Characteristic "SJK1" Client Characteristic Configuration Descriptor
//static gattCharCfg_t *bs_SJK1Config;







/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t Sjk_ServiceAttrTbl[] =
{
  // Sjk_Service Service Declaration
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8_t *)&SjkServiceDecl
  },
    // SJK0 Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &bs_SJK0Props
    },
      // SJK0 Characteristic Value
      {
        { ATT_UUID_SIZE, bs_SJK0UUID },
        GATT_PERMIT_READ,
        0,
        bs_SJK0Val
      },
      // SJK0 CCCD
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&bs_SJK0Config
      },

    // SJK1 Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &bs_SJK1Props
    },
      // SJK1 Characteristic Value
      {
        { ATT_UUID_SIZE, bs_SJK1UUID },
        GATT_PERMIT_READ,
        0,
        bs_SJK1Val
      },
   

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t Sjk_Service_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t Sjk_Service_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t Sjk_ServiceCBs =
{
  Sjk_Service_ReadAttrCB,  // Read callback function pointer
  Sjk_Service_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*
 * SjkService_AddService- Initializes the SjkService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t SjkService_AddService( uint8_t rspTaskId )
{
  uint8_t status;

  // Allocate Client Characteristic Configuration table
  bs_SJK0Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
  if ( bs_SJK0Config == NULL )
  {
    return ( bleMemAllocError );
  }
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, bs_SJK0Config );
  

  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService( Sjk_ServiceAttrTbl,
                                        GATT_NUM_ATTRS( Sjk_ServiceAttrTbl ),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &Sjk_ServiceCBs );
  Log_info1("Registered service, %d attributes", (IArg)GATT_NUM_ATTRS( Sjk_ServiceAttrTbl ));
  bs_icall_rsp_task_id = rspTaskId;

  return ( status );
}

/*
 * SjkService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t SjkService_RegisterAppCBs( SjkServiceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    pAppCBs = appCallbacks;
    Log_info1("Registered callbacks to application. Struct %p", (IArg)appCallbacks);
    return ( SUCCESS );
  }
  else
  {
    Log_warning0("Null pointer given for app callbacks.");
    return ( FAILURE );
  }
}


static int yyy = 0;
/*
 * SjkService_SetParameter - Set a SjkService parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t SjkService_SetParameter( uint8_t param, uint16_t len, void *value )
{
  bStatus_t ret = SUCCESS;
  uint8_t  *pAttrVal;
  uint16_t *pValLen;
  uint16_t valMinLen;
  uint16_t valMaxLen;
  uint8_t sendNotiInd = FALSE;
  gattCharCfg_t *attrConfig;
  uint8_t needAuth;

  switch ( param )
  {
    case BS_SJK0_ID:
      pAttrVal  =  bs_SJK0Val;
      pValLen   = &bs_SJK0ValLen;
      valMinLen =  BS_SJK0_LEN_MIN;
      valMaxLen =  BS_SJK0_LEN;

      sendNotiInd = TRUE;
      attrConfig  = bs_SJK0Config;
      needAuth    = FALSE; // Change if authenticated link is required for sending.
      Log_info2("SetParameter : %s len: %d", (IArg)"SJK0", (IArg)len);

      /*
      if(yyy == 0 ) yyy = 1;
      else yyy = 0;

      switch(yyy)
      {
          case 1:
            Log_info0("do case 1");
            for(int i=0; i < BS_SJK1_LEN_MIN; i++) bs_SJK1Val[i]=i;
            break;
          case 0:
            Log_info0("do case 2");
            for(int i=0; i < BS_SJK1_LEN_MIN; i++) bs_SJK1Val[i] = 99 - i;
            break;
      }
      */


      break;

    case BS_SJK1_ID:
      pAttrVal  =  bs_SJK1Val;
      pValLen   = &bs_SJK1ValLen;
      valMinLen =  BS_SJK1_LEN_MIN;
      valMaxLen =  BS_SJK1_LEN;
      Log_info2("SetParameter : %s len: %d", (IArg)"SJK1", (IArg)len);
      break;



    default:
      Log_error1("SetParameter: Parameter #%d not valid.", (IArg)param);
      return INVALIDPARAMETER;
  }

  // Check bounds, update value and send notification or indication if possible.
  if ( len <= valMaxLen && len >= valMinLen )
  {
    memcpy(pAttrVal, value, len);
    *pValLen = len; // Update length for read and get.

    if (sendNotiInd)
    {
      Log_info2("Trying to send noti/ind: connHandle %x, %s",
                (IArg)attrConfig[0].connHandle,
                (IArg)((attrConfig[0].value==0)?"\x1b[33mNoti/ind disabled\x1b[0m" :
                       (attrConfig[0].value==1)?"Notification enabled" :
                                                "Indication enabled"));
      // Try to send notification.
      //sgkim

      GATTServApp_ProcessCharCfg( attrConfig, pAttrVal, needAuth,
                                  Sjk_ServiceAttrTbl, GATT_NUM_ATTRS( Sjk_ServiceAttrTbl ),
                                  bs_icall_rsp_task_id,  Sjk_Service_ReadAttrCB);


    }
  }
  else
  {
    Log_error3("Length outside bounds: Len: %d MinLen: %d MaxLen: %d.", (IArg)len, (IArg)valMinLen, (IArg)valMaxLen);
    ret = bleInvalidRange;
  }

  return ret;
}


/*
 * SjkService_GetParameter - Get a SjkService parameter.
 *
 *    param - Profile parameter ID
 *    len   - pointer to a variable that contains the maximum length that can be written to *value.
              After the call, this value will contain the actual returned length.
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t SjkService_GetParameter( uint8_t param, uint16_t *len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    default:
      Log_error1("GetParameter: Parameter #%d not valid.", (IArg)param);
      ret = INVALIDPARAMETER;
      break;
  }
  return ret;
}

/*********************************************************************
 * @internal
 * @fn          Sjk_Service_findCharParamId
 *
 * @brief       Find the logical param id of an attribute in the service's attr table.
 *
 *              Works only for Characteristic Value attributes and
 *              Client Characteristic Configuration Descriptor attributes.
 *
 * @param       pAttr - pointer to attribute
 *
 * @return      uint8_t paramID (ref sjk_service.h) or 0xFF if not found.
 */
static uint8_t Sjk_Service_findCharParamId(gattAttribute_t *pAttr)
{
  // Is this a Client Characteristic Configuration Descriptor?
  if (ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID == *(uint16_t *)pAttr->type.uuid)
    return Sjk_Service_findCharParamId(pAttr - 1); // Assume the value attribute precedes CCCD and recurse

  // Is this attribute in "SJK0"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, bs_SJK0UUID, pAttr->type.len))
    return BS_SJK0_ID;

  // Is this attribute in "SJK1"?
  else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, bs_SJK1UUID, pAttr->type.len))
    return BS_SJK1_ID;

  else
    return 0xFF; // Not found. Return invalid.
}



/*********************************************************************
 * @fn          Sjk_Service_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t Sjk_Service_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{
  bStatus_t status = SUCCESS;
  uint16_t valueLen;
  uint8_t paramID = 0xFF;

  // Find settings for the characteristic to be read.
  paramID = Sjk_Service_findCharParamId( pAttr );
  switch ( paramID )
  {
    case BS_SJK0_ID:
      valueLen = bs_SJK0ValLen;

      Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                 (IArg)"SJK0",
                 (IArg)connHandle,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for SJK0 can be inserted here */
      break;

    case BS_SJK1_ID:
      valueLen = bs_SJK1ValLen;

      Log_info4("ReadAttrCB : %s connHandle: %d offset: %d method: 0x%02x",
                 (IArg)"SJK1",
                 (IArg)connHandle,
                 (IArg)offset,
                 (IArg)method);
      /* Other considerations for SJK1 can be inserted here */
      break;

    default:
      Log_error0("Attribute was not found.");
      return ATT_ERR_ATTR_NOT_FOUND;
  }

  // Check bounds and return the value
  if ( offset > valueLen )  // Prevent malicious ATT ReadBlob offsets.
  {
    Log_error0("An invalid offset was requested.");
    status = ATT_ERR_INVALID_OFFSET;

  }
  else
  {

    Log_info1("maxLen = %d", maxLen);
    Log_info1("offset = %d", offset);

    *pLen = MIN(maxLen, valueLen - offset);     //Transmit as much as possible
    memcpy(pValue, pAttr->pValue + offset, *pLen);

  }

  return status;
}

/*********************************************************************
 * @fn      Sjk_Service_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t Sjk_Service_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{
  bStatus_t status  = SUCCESS;
  uint8_t   paramID = 0xFF;

  // See if request is regarding a Client Characterisic Configuration
  if (ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID == *(uint16_t *)pAttr->type.uuid)
  {
    Log_info3("WriteAttrCB (CCCD): param: %d connHandle: %d %s",
              (IArg)Sjk_Service_findCharParamId(pAttr),
              (IArg)connHandle,
              (IArg)(method == GATT_LOCAL_WRITE?"- restoring bonded state":"- OTA write"));

    // Allow notification and indication, but do not check if really allowed per CCCD.
    status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                             offset, GATT_CLIENT_CFG_NOTIFY |
                                                     GATT_CLIENT_CFG_INDICATE );
    if (SUCCESS == status && pAppCBs && pAppCBs->pfnCfgChangeCb)
       pAppCBs->pfnCfgChangeCb( connHandle, SJK_SERVICE_SERV_UUID,
                                Sjk_Service_findCharParamId(pAttr), pValue, len );

     return status;
  }

  // Find settings for the characteristic to be written.
  paramID = Sjk_Service_findCharParamId( pAttr );
  switch ( paramID )
  {
    default:
      Log_error0("Attribute was not found.");
      return ATT_ERR_ATTR_NOT_FOUND;
  }
}