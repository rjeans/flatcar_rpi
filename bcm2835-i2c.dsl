DefinitionBlock ("bcm2835-i2c.aml", "SSDT", 2, "RPIFND", "I2C1    ", 0x00000001)
{
    External (\_SB_, DeviceObj)
    Scope (\_SB)
    {
        Device (I2C1)
        {
            Name (_HID, "BCM2841")          // Custom HID (should match your kernel binding)
            Name (_UID, 1)
            Name (_CCA, 0)                  // Cache coherent attribute
            Name (_STA, 0x0F)               // Device enabled

            Name (RBUF, ResourceTemplate ()
            {
                Memory32Fixed (ReadWrite, 0x7E804000, 0x1000)   // I2C1 base address
                Interrupt (ResourceConsumer, Level, ActiveHigh, Shared, ,, ) { 81 } // IRQ 81 = I2C1 on RPi4
            })

            Method (_CRS, 0, Serialized)
            {
                Return (RBUF)
            }

            // Device Properties via _DSD
            Name (_DSD, Package ()
            {
                ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"), // Device Properties UUID
                Package ()
                {
                    // Pinctrl: GPIO 2 and 3 to alt0
                    Package () { "pinctrl-0", Package ()
                        {
                            Package ()
                            {
                                Package () { "pins", Package () { 2, 3 } },
                                Package () { "function", "alt0" }
                            }
                        }
                    },
                    Package () { "pinctrl-names", Package () { "default" } }
                }
            })
        }
    }
}
