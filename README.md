# Subaru BRZ HV ECU

This is the HV ECU for my Subaru BRZ EV conversion. It manages communication with the charge port controller (Foccci), a Tesla Model 3 PCS OpenInverter controller, Taycan battery modules, and an IVT-S current shunt

## Specs
- 16 Taycan modules (8 on each BCC chain)
- 3 CAN bus ports (only 1 currently used)
- Contactor control for up to 4 contactors (two to control AC charge path, two for HV system)
```

## Building and Flashing

This project uses PlatformIO:

```bash
# Build the project
pio run

# Upload to board
pio run --target upload

# Open serial monitor
pio device monitor
```

## License

[Your License Here]

## Contributing

[Your Contributing Guidelines Here]