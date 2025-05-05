/*
 * Intel ACPI Component Architecture
 * AML/ASL+ Disassembler version 20240322 (64-bit version)
 * Copyright (c) 2000 - 2023 Intel Corporation
 * 
 * Disassembling to symbolic ASL+ operators
 *
 * Disassembly of bcm2835-i2c.dsl
 *
 * Original Table Header:
 *     Signature        "SSDT"
 *     Length           0x00000102 (258)
 *     Revision         0x02
 *     Checksum         0x62
 *     OEM ID           "RPIFND"
 *     OEM Table ID     "I2C1    "
 *     OEM Revision     0x00000001 (1)
 *     Compiler ID      "INTL"
 *     Compiler Version 0x20240322 (539231010)
 */
DefinitionBlock ("", "SSDT", 2, "RPIFND", "I2C1    ", 0x00000001)
{
    External (_SB_, DeviceObj)

    Scope (\_SB)
    {
        Device (I2C1)
        {
            Name (_HID, "PRP0001")  // _HID: Hardware ID
            Name (_UID, One)  // _UID: Unique ID
            Name (_CCA, Zero)  // _CCA: Cache Coherency Attribute
            Name (_STA, 0x0F)  // _STA: Status
            Name (RBUF, ResourceTemplate ()
            {
                Memory32Fixed (ReadWrite,
                    0x7E804000,         // Address Base
                    0x00001000,         // Address Length
                    )
                Interrupt (ResourceConsumer, Level, ActiveHigh, Shared, ,, )
                {
                    0x00000051,
                }
            })
            Method (_CRS, 0, Serialized)  // _CRS: Current Resource Settings
            {
                Return (RBUF) /* \_SB_.I2C1.RBUF */
            }

            Name (_DSD, Package (0x02)  // _DSD: Device-Specific Data
            {
                ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301") /* Device Properties for _DSD */, 
                Package (0x02)
                {
                    Package (0x02)
                    {
                        "pinctrl-0", 
                        Package (0x01)
                        {
                            Package (0x02)
                            {
                                Package (0x02)
                                {
                                    "pins", 
                                    Package (0x02)
                                    {
                                        0x02, 
                                        0x03
                                    }
                                }, 

                                Package (0x02)
                                {
                                    "function", 
                                    "alt0"
                                }
                            }
                        }
                    }, 

                    Package (0x02)
                    {
                        "pinctrl-names", 
                        Package (0x01)
                        {
                            "default"
                        }
                    }
                }
            })
        }
    }
}

