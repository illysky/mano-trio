
#include "fusb30X.h"
#include "platform.h"

FSC_BOOL DeviceWrite(void* dev, FSC_U8 regAddr,FSC_U8 length, FSC_U8* data)
{
    return platform_i2c_write(dev, FUSB300AddrLength, length,length, FUSB300IncSize, regAddr, data);
}

FSC_BOOL DeviceRead(void* dev, FSC_U8 regAddr,FSC_U8 length, FSC_U8* data)
{
    return platform_i2c_read(dev, FUSB300AddrLength, length, length, FUSB300IncSize, regAddr, data);
}
