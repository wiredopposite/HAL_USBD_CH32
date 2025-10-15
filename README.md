# HAL_USBD_CH32
USB device stack and hardware abstraction for the CH32V20x full speed peripheral

This is a very basic implementation and currently lacks several features:
- Double buffered endpoint handling
- Bulk transfer handling
- Support for the high speed USB peripheral
- Alternate interface settings
- Device level feature requests
- Support for models other than the CH32V20x (or others that use the same full speed peripheral)