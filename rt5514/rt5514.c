#include "rt5514.h"
#include "registers.h"

#define bool int

static ULONG Rt5514DebugLevel = 100;
static ULONG Rt5514DebugCatagories = DBG_INIT || DBG_PNP || DBG_IOCTL;

NTSTATUS
DriverEntry(
	__in PDRIVER_OBJECT  DriverObject,
	__in PUNICODE_STRING RegistryPath
)
{
	NTSTATUS               status = STATUS_SUCCESS;
	WDF_DRIVER_CONFIG      config;
	WDF_OBJECT_ATTRIBUTES  attributes;

	Rt5514Print(DEBUG_LEVEL_INFO, DBG_INIT,
		"Driver Entry\n");

	WDF_DRIVER_CONFIG_INIT(&config, Rt5514EvtDeviceAdd);

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);

	//
	// Create a framework driver object to represent our driver.
	//

	status = WdfDriverCreate(DriverObject,
		RegistryPath,
		&attributes,
		&config,
		WDF_NO_HANDLE
	);

	if (!NT_SUCCESS(status))
	{
		Rt5514Print(DEBUG_LEVEL_ERROR, DBG_INIT,
			"WdfDriverCreate failed with status 0x%x\n", status);
	}

	return status;
}

#define RT5514_DSP_MAPPING			0x18000000

static NTSTATUS rt5514_reg_write(PRT5514_CONTEXT pDevice, uint16_t reg, uint32_t data)
{
	uint32_t reg32 = reg | RT5514_DSP_MAPPING;
	uint32_t rawdata[2];
	rawdata[0] = RtlUlongByteSwap(reg32);
	rawdata[1] = RtlUlongByteSwap(data);
	return SpbWriteDataSynchronously(&pDevice->I2CContext, rawdata, sizeof(rawdata));
}
static NTSTATUS rt5514_reg_read(PRT5514_CONTEXT pDevice, uint16_t reg, uint32_t* data)
{
	uint32_t reg32 = RT5514_DSP_MAPPING | reg;
	uint32_t reg_swap = RtlUlongByteSwap(reg32);
	uint32_t data_swap = 0;
	NTSTATUS ret = SpbXferDataSynchronously(&pDevice->I2CContext, &reg_swap, sizeof(uint32_t), &data_swap, sizeof(uint32_t));
	*data = RtlUlongByteSwap(data_swap);
	return ret;
}

static NTSTATUS rt5514_reg_update(
	_In_ PRT5514_CONTEXT pDevice,
	uint16_t reg,
	uint32_t mask,
	uint32_t val
) {
	uint32_t tmp = 0, orig = 0;

	NTSTATUS status = rt5514_reg_read(pDevice, reg, &orig);
	if (!NT_SUCCESS(status)) {
		return status;
	}

	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig) {
		status = rt5514_reg_write(pDevice, reg, tmp);
	}
	return status;
}

struct reg {
	UINT32 reg;
	UINT32 val;
};


static NTSTATUS rt5514_reg_burstWrite(PRT5514_CONTEXT pDevice, struct reg* regs, int regCount) {
	NTSTATUS status = STATUS_NO_MEMORY;
	for (int i = 0; i < regCount; i++) {
		struct reg* regToSet = &regs[i];

		uint32_t reg32 = regToSet->reg;
		uint32_t rawdata[2];
		rawdata[0] = RtlUlongByteSwap(reg32);
		rawdata[1] = RtlUlongByteSwap(regToSet->val);

		status = SpbWriteDataSynchronously(&pDevice->I2CContext, rawdata, sizeof(rawdata));
		if (!NT_SUCCESS(status)) {
			return status;
		}
	}
	return status;
}

NTSTATUS
StartCodec(
	PRT5514_CONTEXT pDevice
) {
	NTSTATUS status = STATUS_SUCCESS;

	
	uint32_t val = 0;
	rt5514_reg_read(pDevice, RT5514_VENDOR_ID2, &val);
	if (val != RT5514_DEVICE_ID) {
		rt5514_reg_read(pDevice, RT5514_VENDOR_ID2, &val);
		if (val != RT5514_DEVICE_ID) {
			return STATUS_INVALID_DEVICE_STATE;
		}
	}


	struct reg rt5514_i2c_patch[] = {
		{0x1800101c, 0x00000000},
		{0x18001100, 0x0000031f},
		{0x18001104, 0x00000007},
		{0x18001108, 0x00000000},
		{0x1800110c, 0x00000000},
		{0x18001110, 0x00000000},
		{0x18001114, 0x00000001},
		{0x18001118, 0x00000000},
		{0x18002f08, 0x00000006},
		{0x18002f00, 0x00055149},
		{0x18002f00, 0x0005514b},
		{0x18002f00, 0x00055149},
		{0xfafafafa, 0x00000001},
		{0x18002f10, 0x00000001},
		{0x18002f10, 0x00000000},
		{0x18002f10, 0x00000001},
		{0xfafafafa, 0x00000001},
		{0x18002000, 0x000010ec},
		{0xfafafafa, 0x00000000},
	};

	status = rt5514_reg_burstWrite(pDevice, rt5514_i2c_patch, sizeof(rt5514_i2c_patch) / sizeof(struct reg));
	if (!NT_SUCCESS(status)) {
		return status;
	}

	static struct reg rt5514_patch[] = {
	{RT5514_DIG_IO_CTRL | RT5514_DSP_MAPPING,		0x00000040},
	{RT5514_CLK_CTRL1 | RT5514_DSP_MAPPING,		0x38020041},
	{RT5514_SRC_CTRL | RT5514_DSP_MAPPING,		0x44000eee},
	{RT5514_ANA_CTRL_LDO10 | RT5514_DSP_MAPPING,		0x00028604},
	{RT5514_ANA_CTRL_ADCFED | RT5514_DSP_MAPPING,	0x00000800},
	{RT5514_ASRC_IN_CTRL1 | RT5514_DSP_MAPPING,		0x00000003},
	{RT5514_DOWNFILTER0_CTRL3 | RT5514_DSP_MAPPING,	0x10000342},
	{RT5514_DOWNFILTER1_CTRL3 | RT5514_DSP_MAPPING,	0x10000342},
	};

	status = rt5514_reg_burstWrite(pDevice, rt5514_patch, sizeof(rt5514_patch) / sizeof(struct reg));
	if (!NT_SUCCESS(status)) {
		return status;
	}

	// Set TDM slot
	{
		unsigned int val = RT5514_TDM_MODE;
		unsigned int val2 = 0;

		//tx mask 0xF
		val2 = RT5514_TDM_DOCKING_MODE | RT5514_TDM_DOCKING_VALID_CH4 |
			RT5514_TDM_DOCKING_START_SLOT0;

		//8 slots
		val |= RT5514_TDMSLOT_SEL_RX_8CH | RT5514_TDMSLOT_SEL_TX_8CH;

		//16 slot width

		rt5514_reg_update(pDevice, RT5514_I2S_CTRL1, RT5514_TDM_MODE |
			RT5514_TDMSLOT_SEL_RX_MASK | RT5514_TDMSLOT_SEL_TX_MASK |
			RT5514_CH_LEN_RX_MASK | RT5514_CH_LEN_TX_MASK |
			RT5514_TDM_MODE2, val);
		rt5514_reg_update(pDevice, RT5514_I2S_CTRL2,
			RT5514_TDM_DOCKING_MODE | RT5514_TDM_DOCKING_VALID_CH_MASK |
			RT5514_TDM_DOCKING_START_MASK, val2);
	}

	//Set sysclk + PLL

	rt5514_reg_write(pDevice, RT5514_CLK_CTRL1, 0x39820342);
	rt5514_reg_write(pDevice, RT5514_CLK_CTRL2, 0x00030112);

	//set DAI fmt

	rt5514_reg_update(pDevice, RT5514_I2S_CTRL1, RT5514_I2S_DF_MASK | RT5514_I2S_BP_MASK | RT5514_I2S_LR_MASK, RT5514_I2S_DF_PCM_B);

	//set capture volumes

	rt5514_reg_write(pDevice, RT5514_DOWNFILTER0_CTRL1, 0x0002046f);
	rt5514_reg_write(pDevice, RT5514_DOWNFILTER0_CTRL2, 0x0002046f);

	rt5514_reg_write(pDevice, RT5514_DOWNFILTER1_CTRL1, 0x0002046f);
	rt5514_reg_write(pDevice, RT5514_DOWNFILTER1_CTRL2, 0x0002046f);

	pDevice->DevicePoweredOn = TRUE;
	return status;
}

NTSTATUS
StopCodec(
	PRT5514_CONTEXT pDevice
) {
	NTSTATUS status = STATUS_SUCCESS;

	pDevice->DevicePoweredOn = FALSE;
	return status;
}

int CsAudioArg2 = 1;

VOID
CSAudioRegisterEndpoint(
	PRT5514_CONTEXT pDevice
) {
	CsAudioArg arg;
	RtlZeroMemory(&arg, sizeof(CsAudioArg));
	arg.argSz = sizeof(CsAudioArg);
	arg.endpointType = CSAudioEndpointTypeMicArray;
	arg.endpointRequest = CSAudioEndpointRegister;
	ExNotifyCallback(pDevice->CSAudioAPICallback, &arg, &CsAudioArg2);

	RtlZeroMemory(&arg, sizeof(CsAudioArg));
	arg.argSz = sizeof(CsAudioArg);
	arg.endpointType = CSAudioEndpointTypeMicArray;
	arg.endpointRequest = CSAudioEndpointOverrideFormat;
	arg.formatOverride.force32BitOutputContainer = TRUE;
	ExNotifyCallback(pDevice->CSAudioAPICallback, &arg, &CsAudioArg2);
}

VOID
CsAudioCallbackFunction(
	IN PRT5514_CONTEXT pDevice,
	CsAudioArg* arg,
	PVOID Argument2
) {
	if (!pDevice) {
		return;
	}

	if (Argument2 == &CsAudioArg2) {
		return;
	}

	pDevice->CSAudioManaged = TRUE;

	CsAudioArg localArg;
	RtlZeroMemory(&localArg, sizeof(CsAudioArg));
	RtlCopyMemory(&localArg, arg, min(arg->argSz, sizeof(CsAudioArg)));

	if (localArg.endpointType == CSAudioEndpointTypeDSP && localArg.endpointRequest == CSAudioEndpointRegister) {
		CSAudioRegisterEndpoint(pDevice);
	}
	else if (localArg.endpointType != CSAudioEndpointTypeMicArray) {
		return;
	}

	if (localArg.endpointRequest == CSAudioEndpointStop) {
		StopCodec(pDevice);
	}
	if (localArg.endpointRequest == CSAudioEndpointStart) {
		StartCodec(pDevice);
	}
}

NTSTATUS
OnPrepareHardware(
	_In_  WDFDEVICE     FxDevice,
	_In_  WDFCMRESLIST  FxResourcesRaw,
	_In_  WDFCMRESLIST  FxResourcesTranslated
)
/*++

Routine Description:

This routine caches the SPB resource connection ID.

Arguments:

FxDevice - a handle to the framework device object
FxResourcesRaw - list of translated hardware resources that
the PnP manager has assigned to the device
FxResourcesTranslated - list of raw hardware resources that
the PnP manager has assigned to the device

Return Value:

Status

--*/
{
	PRT5514_CONTEXT pDevice = GetDeviceContext(FxDevice);
	BOOLEAN fSpbResourceFound = FALSE;
	NTSTATUS status = STATUS_INSUFFICIENT_RESOURCES;

	UNREFERENCED_PARAMETER(FxResourcesRaw);

	//
	// Parse the peripheral's resources.
	//

	ULONG resourceCount = WdfCmResourceListGetCount(FxResourcesTranslated);

	for (ULONG i = 0; i < resourceCount; i++)
	{
		PCM_PARTIAL_RESOURCE_DESCRIPTOR pDescriptor;
		UCHAR Class;
		UCHAR Type;

		pDescriptor = WdfCmResourceListGetDescriptor(
			FxResourcesTranslated, i);

		switch (pDescriptor->Type)
		{
		case CmResourceTypeConnection:
			//
			// Look for I2C or SPI resource and save connection ID.
			//
			Class = pDescriptor->u.Connection.Class;
			Type = pDescriptor->u.Connection.Type;
			if (Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL &&
				Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)
			{
				if (fSpbResourceFound == FALSE)
				{
					status = STATUS_SUCCESS;
					pDevice->I2CContext.I2cResHubId.LowPart = pDescriptor->u.Connection.IdLowPart;
					pDevice->I2CContext.I2cResHubId.HighPart = pDescriptor->u.Connection.IdHighPart;
					fSpbResourceFound = TRUE;
				}
				else
				{
				}
			}
			break;
		default:
			//
			// Ignoring all other resource types.
			//
			break;
		}
	}

	//
	// An SPB resource is required.
	//

	if (fSpbResourceFound == FALSE)
	{
		status = STATUS_NOT_FOUND;
	}

	status = SpbTargetInitialize(FxDevice, &pDevice->I2CContext);

	if (!NT_SUCCESS(status))
	{
		return status;
	}

	return status;
}

NTSTATUS
OnReleaseHardware(
	_In_  WDFDEVICE     FxDevice,
	_In_  WDFCMRESLIST  FxResourcesTranslated
)
/*++

Routine Description:

Arguments:

FxDevice - a handle to the framework device object
FxResourcesTranslated - list of raw hardware resources that
the PnP manager has assigned to the device

Return Value:

Status

--*/
{
	PRT5514_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	UNREFERENCED_PARAMETER(FxResourcesTranslated);

	SpbTargetDeinitialize(FxDevice, &pDevice->I2CContext);

	if (pDevice->CSAudioAPICallbackObj) {
		ExUnregisterCallback(pDevice->CSAudioAPICallbackObj);
		pDevice->CSAudioAPICallbackObj = NULL;
	}

	if (pDevice->CSAudioAPICallback) {
		ObfDereferenceObject(pDevice->CSAudioAPICallback);
		pDevice->CSAudioAPICallback = NULL;
	}

	return status;
}

NTSTATUS
OnSelfManagedIoInit(
	_In_
	WDFDEVICE FxDevice
) {
	PRT5514_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	// CS Audio Callback

	UNICODE_STRING CSAudioCallbackAPI;
	RtlInitUnicodeString(&CSAudioCallbackAPI, L"\\CallBack\\CsAudioCallbackAPI");


	OBJECT_ATTRIBUTES attributes;
	InitializeObjectAttributes(&attributes,
		&CSAudioCallbackAPI,
		OBJ_KERNEL_HANDLE | OBJ_OPENIF | OBJ_CASE_INSENSITIVE | OBJ_PERMANENT,
		NULL,
		NULL
	);
	status = ExCreateCallback(&pDevice->CSAudioAPICallback, &attributes, TRUE, TRUE);
	if (!NT_SUCCESS(status)) {

		return status;
	}

	pDevice->CSAudioAPICallbackObj = ExRegisterCallback(pDevice->CSAudioAPICallback,
		CsAudioCallbackFunction,
		pDevice
	);
	if (!pDevice->CSAudioAPICallbackObj) {

		return STATUS_NO_CALLBACK_ACTIVE;
	}

	CSAudioRegisterEndpoint(pDevice);

	return status;
}

NTSTATUS
OnD0Entry(
	_In_  WDFDEVICE               FxDevice,
	_In_  WDF_POWER_DEVICE_STATE  FxPreviousState
)
/*++

Routine Description:

This routine allocates objects needed by the driver.

Arguments:

FxDevice - a handle to the framework device object
FxPreviousState - previous power state

Return Value:

Status

--*/
{
	UNREFERENCED_PARAMETER(FxPreviousState);

	PRT5514_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	status = StartCodec(pDevice);

	return status;
}

NTSTATUS
OnD0Exit(
	_In_  WDFDEVICE               FxDevice,
	_In_  WDF_POWER_DEVICE_STATE  FxPreviousState
)
/*++

Routine Description:

This routine destroys objects needed by the driver.

Arguments:

FxDevice - a handle to the framework device object
FxPreviousState - previous power state

Return Value:

Status

--*/
{
	UNREFERENCED_PARAMETER(FxPreviousState);

	PRT5514_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	status = StopCodec(pDevice);

	return STATUS_SUCCESS;
}

NTSTATUS
Rt5514EvtDeviceAdd(
	IN WDFDRIVER       Driver,
	IN PWDFDEVICE_INIT DeviceInit
)
{
	NTSTATUS                      status = STATUS_SUCCESS;
	WDF_IO_QUEUE_CONFIG           queueConfig;
	WDF_OBJECT_ATTRIBUTES         attributes;
	WDFDEVICE                     device;
	WDFQUEUE                      queue;
	PRT5514_CONTEXT               devContext;

	UNREFERENCED_PARAMETER(Driver);

	PAGED_CODE();

	Rt5514Print(DEBUG_LEVEL_INFO, DBG_PNP,
		"Rt5514EvtDeviceAdd called\n");

	{
		WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;
		WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);

		pnpCallbacks.EvtDevicePrepareHardware = OnPrepareHardware;
		pnpCallbacks.EvtDeviceReleaseHardware = OnReleaseHardware;
		pnpCallbacks.EvtDeviceSelfManagedIoInit = OnSelfManagedIoInit;
		pnpCallbacks.EvtDeviceD0Entry = OnD0Entry;
		pnpCallbacks.EvtDeviceD0Exit = OnD0Exit;

		WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);
	}

	//
	// Setup the device context
	//

	WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, RT5514_CONTEXT);

	//
	// Create a framework device object.This call will in turn create
	// a WDM device object, attach to the lower stack, and set the
	// appropriate flags and attributes.
	//

	status = WdfDeviceCreate(&DeviceInit, &attributes, &device);

	if (!NT_SUCCESS(status))
	{
		Rt5514Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfDeviceCreate failed with status code 0x%x\n", status);

		return status;
	}

	{
		WDF_DEVICE_STATE deviceState;
		WDF_DEVICE_STATE_INIT(&deviceState);

		deviceState.NotDisableable = WdfFalse;
		WdfDeviceSetDeviceState(device, &deviceState);
	}

	WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&queueConfig, WdfIoQueueDispatchParallel);

	queueConfig.EvtIoInternalDeviceControl = Rt5514EvtInternalDeviceControl;

	status = WdfIoQueueCreate(device,
		&queueConfig,
		WDF_NO_OBJECT_ATTRIBUTES,
		&queue
	);

	if (!NT_SUCCESS(status))
	{
		Rt5514Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfIoQueueCreate failed 0x%x\n", status);

		return status;
	}

	//
	// Create manual I/O queue to take care of hid report read requests
	//

	devContext = GetDeviceContext(device);

	devContext->FxDevice = device;

	WDF_IO_QUEUE_CONFIG_INIT(&queueConfig, WdfIoQueueDispatchManual);

	queueConfig.PowerManaged = WdfFalse;

	status = WdfIoQueueCreate(device,
		&queueConfig,
		WDF_NO_OBJECT_ATTRIBUTES,
		&devContext->ReportQueue
	);

	if (!NT_SUCCESS(status))
	{
		Rt5514Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfIoQueueCreate failed 0x%x\n", status);

		return status;
	}

	return status;
}

VOID
Rt5514EvtInternalDeviceControl(
	IN WDFQUEUE     Queue,
	IN WDFREQUEST   Request,
	IN size_t       OutputBufferLength,
	IN size_t       InputBufferLength,
	IN ULONG        IoControlCode
)
{
	NTSTATUS            status = STATUS_SUCCESS;
	WDFDEVICE           device;
	PRT5514_CONTEXT     devContext;

	UNREFERENCED_PARAMETER(OutputBufferLength);
	UNREFERENCED_PARAMETER(InputBufferLength);

	device = WdfIoQueueGetDevice(Queue);
	devContext = GetDeviceContext(device);

	switch (IoControlCode)
	{
	default:
		status = STATUS_NOT_SUPPORTED;
		break;
	}

	WdfRequestComplete(Request, status);

	return;
}