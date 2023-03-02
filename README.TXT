multi_role_init:
- multi_role_processGapMsg
-- GAP_DEVICE_INIT_DONE_EVENT
--- multi_role_advertInit
---- GapAdv_enable
--- multi_role_scanInit
---- GapScan_enable

GAP_LINK_ESTABLISHED_EVENT:
- turn off adv (max 1 conn)
- turn off scanning (GapScan_disable)
- sets numConn == 1

MR_EVT_SCAN_DISABLED:
- occurs after scan and saves scans
- re-enables scan (GapScan_enable) unless numConn > 0

to keep scannning:
GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);



[ ] rm toggleLED (and timer)?


/*** from end ***/

/*********************************************************************
* @fn      multi_role_doDiscoverDevices
*
* @brief   Respond to user input to start scanning
*
* @param   index - not used
*
* @return  TRUE since there is no callback to use this value
*/
bool multi_role_doDiscoverDevices(uint8_t index)
{
  (void) index;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // The stack does not need to record advertising reports
  // since the application will filter them by Service UUID and save.

  // Reset number of scan results to 0 before starting scan
  numScanRes = 0;
  GapScan_enable(0, DEFAULT_SCAN_DURATION, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  // Scanning for DEFAULT_SCAN_DURATION x 10 ms.
  // Let the stack record the advertising reports as many as up to DEFAULT_MAX_SCAN_RES.
  GapScan_enable(0, DEFAULT_SCAN_DURATION, DEFAULT_MAX_SCAN_RES);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
  // Enable only "Stop Discovering" and disable all others in the main menu
  tbm_setItemStatus(&mrMenuMain, MR_ITEM_STOPDISC,
                    (MR_ITEM_ALL & ~MR_ITEM_STOPDISC));

  return (true);
}

/*********************************************************************
 * @fn      multi_role_doCancelConnecting
 *
 * @brief   Cancel on-going connection attempt
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool multi_role_doCancelConnecting(uint8_t index)
{
  (void) index;

#ifdef __GNUC__
  GapInit_cancelConnect("");
#else
  GapInit_cancelConnect();
#endif // __GNUC__

  return (true);
}

/*********************************************************************
* @fn      multi_role_doConnect
*
* @brief   Respond to user input to form a connection
*
* @param   index - index as selected from the mrMenuConnect
*
* @return  TRUE since there is no callback to use this value
*/
bool multi_role_doConnect(uint8_t index)
{
  // Temporarily disable advertising
  GapAdv_disable(advHandle);

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
  GapInit_connect(scanList[index].addrType & MASK_ADDRTYPE_ID,
                  scanList[index].addr, mrInitPhy, 0);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
  GapScan_Evt_AdvRpt_t advRpt;

  GapScan_getAdvReport(index, &advRpt);

  GapInit_connect(advRpt.addrType & MASK_ADDRTYPE_ID,
                  advRpt.addr, mrInitPhy, 0);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

  // Re-enable advertising
  GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

  // Enable only "Cancel Connecting" and disable all others in the main menu
  tbm_setItemStatus(&mrMenuMain, MR_ITEM_CANCELCONN,
                    (MR_ITEM_ALL & ~MR_ITEM_CANCELCONN));

  Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Connecting...");

  tbm_goTo(&mrMenuMain);

  return (true);
}

/*********************************************************************
 * @fn      multi_role_doSelectConn
 *
 * @brief   Select a connection to communicate with
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool multi_role_doSelectConn(uint8_t index)
{
  uint32_t itemsToDisable = MR_ITEM_NONE;

  // index cannot be equal to or greater than MAX_NUM_BLE_CONNS
  MULTIROLE_ASSERT(index < MAX_NUM_BLE_CONNS);

  mrConnHandle  = connList[index].connHandle;

  if (connList[index].charHandle == 0)
  {
    // Initiate service discovery
    multi_role_enqueueMsg(MR_EVT_SVC_DISC, NULL);

    // Diable GATT Read/Write until simple service is found
    itemsToDisable = MR_ITEM_GATTREAD | MR_ITEM_GATTWRITE;
  }

  // Set the menu title and go to this connection's context
  TBM_SET_TITLE(&mrMenuPerConn, TBM_GET_ACTION_DESC(&mrMenuSelectConn, index));

  tbm_setItemStatus(&mrMenuPerConn, MR_ITEM_NONE, itemsToDisable);

  // Clear non-connection-related message
  Display_clearLine(dispHandle, MR_ROW_NON_CONN);

  tbm_goTo(&mrMenuPerConn);

  return (true);
}

/*********************************************************************
 * @fn      multi_role_doGattRead
 *
 * @brief   GATT Read
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool multi_role_doGattRead(uint8_t index)
{
  attReadReq_t req;
  uint8_t connIndex = multi_role_getConnIndex(mrConnHandle);

  // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
  MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  req.handle = connList[connIndex].charHandle;
  GATT_ReadCharValue(mrConnHandle, &req, selfEntity);

  return (true);
}

/*********************************************************************
 * @fn      multi_role_doGattWrite
 *
 * @brief   GATT Write
 *
 * @param   index - item index from the menu
 *
 * @return  always true
 */
bool multi_role_doGattWrite(uint8_t index)
{
  status_t status;
  uint8_t charVals[4] = { 0x00, 0x55, 0xAA, 0xFF }; // Should be consistent with
                                                    // those in scMenuGattWrite
  attWriteReq_t req;

  req.pValue = GATT_bm_alloc(mrConnHandle, ATT_WRITE_REQ, 1, NULL);

  if ( req.pValue != NULL )
  {
    uint8_t connIndex = multi_role_getConnIndex(mrConnHandle);

    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    req.handle = connList[connIndex].charHandle;
    req.len = 1;
    charVal = charVals[index];
    req.pValue[0] = charVal;
    req.sig = 0;
    req.cmd = 0;

    status = GATT_WriteCharValue(mrConnHandle, &req, selfEntity);
    if ( status != SUCCESS )
    {
      GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
    }
  }

  return (true);
}

/*********************************************************************
* @fn      multi_role_doDisconnect
*
* @brief   Respond to user input to terminate a connection
*
* @param   index - index as selected from the mrMenuConnUpdate
*
* @return  always true
*/
bool multi_role_doDisconnect(uint8_t index)
{
  (void) index;

  // Disconnect
  GAP_TerminateLinkReq(mrConnHandle, HCI_DISCONNECT_REMOTE_USER_TERM);

  return (true);
}
