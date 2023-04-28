* use internal bias RF

- JUXTA_EVT_CONNECTED (&clkJuxtaCentralTimeout)
-- multi_role_startSvcDiscovery
--- multi_role_processGATTDiscEvent
---- JUXTA_EVT_WRITE_GATT, JUXTA_EVT_CENTRAL_TIMEOUT (disconnect)

multi_role_init:
- multi_role_processGapMsg
-- GAP_DEVICE_INIT_DONE_EVENT
--- multi_role_advertInit()
--- multi_role_scanInit()

MR_EVT_SCAN_DISABLED:
- occurs after scan and saves scans
- re-enables scan (GapScan_enable) unless numConn > 0

To duty cycle scanning, see event: MR_EVT_SCAN_DISABLED
Can duty cycle adv using: GapAdv_enable > GAP_ADV_ENABLE_OPTIONS_USE_DURATION
- But where does it signal that it turned off?
-- see: multi_role_processAdvEvent

[ ] logScan could go into multi_role_addScanInfo