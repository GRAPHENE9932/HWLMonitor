#include "usb_hid.h"
#include <stm32f042x6.h>
#include <stdint.h>
#include <stdbool.h>

// These values must be defined in the linker script.
uint16_t _susb_sram;
uint16_t _eusb_sram;

// Converts the absolute memory address inside USB SRAM into address local to USB SRAM.
#define USB_SRAM_ADDR(addr) (uint16_t)((uintptr_t)addr - (uintptr_t)&_susb_sram)
#define MIN(x, y) ((x) < (y) ? (x) : (y))

enum desc_type : uint8_t {
    DEVICE = 1,
    CONFIGURATION = 2,
    STRING = 3,
    INTERFACE = 4,
    ENDPOINT = 5,
    DEVICE_QUALIFIER = 6,
    OTHER_SPEED_CONFIGURATION = 7,
    INTERFACE_POWER = 8,
    HID = 0x21,
    HID_REPORT = 0x22,
};

enum req_type : uint8_t {
    GET_STATUS = 0,
    CLEAR_FEATURE = 1,
    SET_FEATURE = 3,
    SET_ADDRESS = 5,
    GET_DESCRIPTOR = 6,
    SET_DESCRIPTOR = 7,
    GET_CONFIGURATION = 8,
    SET_CONFIGURATION = 9,
    GET_INTERFACE = 10,
    SET_INTERFACE = 11,
    SYNCH_FRAME = 12,
};

struct __attribute__((packed)) {
    uint16_t addr_tx;
    uint16_t count_tx;
    uint16_t addr_rx;
    uint16_t count_rx;
} typedef usb_btable_entry;

struct __attribute__((packed)) {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} typedef setup_packet;

struct __attribute__((packed)) {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} typedef device_descriptor;

struct __attribute__((packed)) {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
} typedef configuration_descriptor;

struct __attribute__((packed)) {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} typedef interface_descriptor;

struct __attribute__((packed)) {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
} typedef endpoint_descriptor;

struct __attribute__((packed)) {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdHID;
    uint8_t bCountryCode;
    uint8_t bNumDescriptors;
    uint8_t bDescriptorType_2;
    uint16_t wDescriptorLength;
} typedef hid_descriptor;

__attribute__((section(".usb_sram"), aligned(8))) static volatile usb_btable_entry usb_btable[8];
__attribute__((section(".usb_sram"), aligned(2))) static volatile uint16_t ep0_tx_buffer[32];
__attribute__((section(".usb_sram"), aligned(2))) static volatile uint16_t ep0_rx_buffer[32];
__attribute__((section(".usb_sram"), aligned(2))) static volatile uint16_t ep1_tx_buffer[32];
__attribute__((section(".usb_sram"), aligned(2))) static volatile uint16_t ep1_rx_buffer[32];

static const device_descriptor DEVICE_DESCRIPTOR = {
    .bLength = sizeof(device_descriptor), // Descriptor size.
    .bDescriptorType = 1, // DEVICE descriptor type.
    .bcdUSB = 0x0200, // USB version 2.0.
    .bDeviceClass = 0x00, // The HID class code is allowed to be used in the configuration descriptor only.
    .bDeviceSubClass = 0x00, // If bDeviceClass is 0, then bDeviceSubClass must be 0 too.
    .bDeviceProtocol = 0x00, // The protocol will be defined in per-configuration basis.
    .bMaxPacketSize0 = 64, // Max packet size for EP0.
    .idVendor = 0x1209, // pid.codes vendor ID.
    .idProduct = 0x000A, // pid.codes Test PID.
    .bcdDevice = 0x0010, // Device version 0.1.
    .iManufacturer = 0, // No string descriptor.
    .iProduct = 0, // No string descriptor.
    .iSerialNumber = 0, // No string descriptor.
    .bNumConfigurations = 1 // Only one configuration.
};

static const configuration_descriptor CONFIGURATION_DESCRIPTOR = {
    .bLength = sizeof(configuration_descriptor), // Descriptor size.
    .bDescriptorType = 2, // CONFIGURATION descriptor type.
    .wTotalLength = sizeof(configuration_descriptor) + sizeof(interface_descriptor) + sizeof(hid_descriptor) + sizeof(endpoint_descriptor) * 2,
    .bNumInterfaces = 1, // Only one interface.
    .bConfigurationValue = 1, // 1st configuration.
    .iConfiguration = 0, // No string descriptor.
    .bmAttributes = 0b10000000, // Not self-powered, no remote wakeup.
    .bMaxPower = 50 // 100mA.
};

static const interface_descriptor INTERFACE_DESCRIPTOR = {
    .bLength = sizeof(interface_descriptor), // Descriptor size.
    .bDescriptorType = 4, // INTERFACE descriptor type.
    .bInterfaceNumber = 0, // 0th interface.
    .bAlternateSetting = 0, // 0th alternate setting.
    .bNumEndpoints = 2, // Two endpoints.
    .bInterfaceClass = 0x03, // HID (Human Interface Device) class code.
    .bInterfaceSubClass = 0x00, // No BIOS support.
    .bInterfaceProtocol = 0x00, // No BIOS support.
    .iInterface = 0 // No string descriptor.
};

static const endpoint_descriptor EP1_IN_DESCRIPTOR = {
    .bLength = sizeof(endpoint_descriptor), // Descriptor size.
    .bDescriptorType = 5, // ENDPOINT descriptor type.
    .bEndpointAddress = 0b10000001, // Endpoint address 1, direction IN.
    .bmAttributes = 0b00000011, // Interrupt transfer type.
    .wMaxPacketSize = 64, // Maximum packet size: 64 bytes (the maximum of full-speed interrupt transfers).
    .bInterval = 1 // Polling interval: 1 frame (1 ms, 1000 Hz).
};

static const endpoint_descriptor EP1_OUT_DESCRIPTOR = {
    .bLength = sizeof(endpoint_descriptor), // Descriptor size.
    .bDescriptorType = 5, // ENDPOINT descriptor type.
    .bEndpointAddress = 0b00000001, // Endpoint address 1, direction OUT.
    .bmAttributes = 0b00000011, // Interrupt transfer type.
    .wMaxPacketSize = 64, // Maximum packet size: 64 bytes (the maximum of full-speed interrupt transfers).
    .bInterval = 1 // Polling interval: 1 frame (1 ms, 1000 Hz).
};

// Possible inputs are all the values in the key enum and all modifiers in
// the key_modifier enum. Only 1 simultaneously pressed non-modifier key is
// supported. All the modifier keys and 1 non-modifier key can be pressed
// at the same time.
static const uint8_t HID_REPORT_DESCRIPTOR[] = {
    // 0b<bTag>'<bType>'<bSize>, <[data]>
    0b0000'01'01, 0x01, // Usage Page (Generic Desktop Controls)
    0b0000'10'01, 0x06, // Usage (Keyboard)
    0b1010'00'01, 0x01, // Collection (Application)
    0b0000'01'01, 0x07, // Usage Page (Keyboard/Keypad)
    0b0001'10'01, 224,  // Usage Minimum (224) (all the modifier keys)
    0b0010'10'01, 231,  // Usage Maximum (231)
    0b0001'01'01, 0,    // Logical Minimum (0)
    0b0010'01'01, 1,    // Logical Maximum (1)
    0b0111'01'01, 1,    // Report Size (1)
    0b1001'01'01, 8,    // Report Count (8)
    0b1000'00'01, 0x02, // Input (Data, Variable, Absolute)
    0b0001'10'01, 0,    // Usage Minimum (0)
    0b0010'10'01, 101,  // Usage Maximum (101)
    0b0001'01'01, 0,    // Logical Minimum (0)
    0b0010'01'01, 101,  // Logical Maximum (101)
    0b0111'01'01, 8,    // Report Size (8)
    0b1001'01'01, 1,    // Report Count (1)
    0b1000'00'01, 0x00, // Input (Data, Array)
    0b1100'00'00,       // End Collection
};
#define HID_REPORT_SIZE 2

static const hid_descriptor HID_DESCRIPTOR = {
    .bLength = sizeof(hid_descriptor), // Descriptor size.
    .bDescriptorType = 0x21, // HID descriptor type.
    .bcdHID = 0x0111, // HID Class Specification version 1.11.
    .bCountryCode = 0, // No localization.
    .bNumDescriptors = 1, // Report descriptor only.
    .bDescriptorType_2 = 0x22, // Report descriptor type.
    .wDescriptorLength = sizeof(HID_REPORT_DESCRIPTOR), // Length of the report descriptor.
};

static struct {
    setup_packet packet;
} control_state;

// Configuration state of 0 means Address or Default state, other values describe the current configuration.
static uint8_t configuration_state = 0;
static volatile uint8_t current_report[HID_REPORT_SIZE] = {0};

// Writing and reading from/to USB SRAM requires special care, as only two-byte
// read/writes are allowed and compiler isn't aware of that.
static void clear_usb_sram(void) {
    for (volatile uint16_t* ptr = &_susb_sram; ptr < &_eusb_sram; ptr++) {
        *ptr = 0;
    }
}

static void copy_from_usb_sram(volatile void* sram_ptr, const volatile void* usb_sram_ptr, uint16_t size) {
    if (size == 0) {
        return;
    }

    // Some aliases for convenient use of pointers further in the function.
    volatile uint8_t* sram = sram_ptr;
    const volatile uint8_t* usb_sram = usb_sram_ptr;

    // Copy the first byte, if the USB_SRAM address is not aligned to 2 bytes.
    if ((uintptr_t)usb_sram % 2 == 1) {
        *(uint8_t*)sram = *(const volatile uint16_t*)(usb_sram - 1) >> 8;
        size--;
        sram++;
        usb_sram++;
    }

    // Main copying.
    for (uint_fast16_t i = 0; i < size / 2; i++) {
        ((uint16_t*)sram)[i] = ((const volatile uint16_t*)usb_sram)[i];
    }

    // Copy the last byte, if there is one byte left.
    if (size % 2 == 1) {
        ((uint8_t*)sram)[size - 1] = ((const volatile uint16_t*)usb_sram)[size / 2] & 0x00FF;
    }
}

static void copy_to_usb_sram(volatile void* usb_sram_ptr, const volatile void* sram_ptr, uint16_t size) {
    if (size == 0) {
        return;
    }

    // Some aliases for convenient use of pointers further in the function.
    volatile uint8_t* usb_sram = usb_sram_ptr;
    const volatile uint8_t* sram = sram_ptr;

    // Copy the first byte, if the USB_SRAM address is not aligned to 2 bytes.
    if ((uintptr_t)usb_sram % 2 == 1) {
        volatile uint16_t* hword_ptr = (volatile uint16_t*)(usb_sram - 1);
        uint16_t hword = (*hword_ptr & 0x00FF) | ((uint16_t)sram[0] << 8);
        *hword_ptr = hword;

        size--;
        sram++;
        usb_sram++;
    }

    // Main copying.
    for (uint_fast16_t i = 0; i < size / 2; i++) {
        //uint16_t hword = ((const uint16_t*)sram)[i];
        uint16_t hword = ((uint16_t)sram[i * 2 + 1] << 8) | sram[i * 2];
        ((volatile uint16_t*)usb_sram)[i] = hword;
    }

    // Copy the last byte, if there is one byte left.
    if (size % 2 == 1) {
        volatile uint16_t* hword_ptr = (volatile uint16_t*)usb_sram + size / 2;
        uint16_t hword = (*hword_ptr & 0xFF00) | sram[size - 1];
        *hword_ptr = hword;
    }
}

// Sets the EPnR bits, specified by <mask>, to <value>.
static void epnr_set(volatile uint16_t* epnr, uint16_t value, uint16_t mask) {
    static const uint16_t ONLY_ZERO_WRITABLE = 0b1000000010000000;
    static const uint16_t TOGGLEABLE = 0b0111000001110000;

    uint16_t epnr_value = *epnr;

    // Simplified complex boolean expression. For toggleable bits
    // it toggles only the bits covered by <mask>, depending on the <value>.
    // For only zero writable bits it writes ones by default and writes zero,
    // if covered by <mask> and the <value> is also zero.
    // For normal bits (which are not only zero writable and not toggleable)
    // it writes the <value> to bits, covered by <mask>.
    uint16_t value_to_write =
        (~ONLY_ZERO_WRITABLE & (
            (~TOGGLEABLE & ((value & mask) | (epnr_value & ~mask))) |
            (TOGGLEABLE & mask & (epnr_value ^ value))
        )) | (ONLY_ZERO_WRITABLE & (
            ~mask | (mask & value)
        ));
    
    *epnr = value_to_write;
}

void usb_init(void) {
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USBRST; // Release reset.
    USB->CNTR &= ~USB_CNTR_PDWN; // Power up USB peripheral.
    // Wait 1 us for the USB peripheral to power up.
    // TODO: Replace with a proper delay.
    for (volatile uint32_t i = 0; i < 48;) {
        i = i + 1;
    }

    USB->CNTR &= ~USB_CNTR_FRES; // Clear USB peripheral reset.
    USB->ISTR = 0; // Clear any pending interrupts.

    clear_usb_sram();
    
    USB->BCDR |= USB_BCDR_DPPU; // Enable pull-up on D+ to signalize about FullSpeed.

    // Enable interrupts.
    USB->CNTR |= USB_CNTR_CTRM | USB_CNTR_ERRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_RESETM | USB_CNTR_SOFM;
    NVIC_EnableIRQ(USB_IRQn);
}

static void handle_reset(void) {
    configuration_state = 0;

    // Initialize btable.
    USB->BTABLE = USB_SRAM_ADDR(usb_btable);
    usb_btable[0].addr_rx = USB_SRAM_ADDR(ep0_rx_buffer);
    usb_btable[0].addr_tx = USB_SRAM_ADDR(ep0_tx_buffer);
    usb_btable[0].count_rx = (1 << 15) | (1 << 10); // Two blocks of size 32 bytes, 64 bytes total.
    usb_btable[0].count_tx = 0; // Amount of bytes for transmission will be set by the handlers themselves later.

    usb_btable[1].addr_rx = USB_SRAM_ADDR(ep1_rx_buffer);
    usb_btable[1].addr_tx = USB_SRAM_ADDR(ep1_tx_buffer);
    usb_btable[1].count_rx = (1 << 15) | (1 << 10); // Two blocks of size 32 bytes, 64 bytes total.
    usb_btable[1].count_tx = 0; // Amount of bytes for transmission will be set by the handlers themselves later.

    // Set the EP0 type to CONTROL, accept RX transactions and NAK TX transactions.
    epnr_set(
        &USB->EP0R,
        USB_EP_CONTROL | USB_EP_TX_VALID | USB_EP_RX_NAK,
        USB_EP_TYPE_MASK | USB_EP_TX_VALID | USB_EP_RX_VALID | USB_EPADDR_FIELD
    );

    // Set the EP1 type to INTERRUPT, accept RX and TX transactions, set EP address to 1.
    epnr_set(
        &USB->EP1R,
        USB_EP_INTERRUPT | USB_EP_TX_VALID | USB_EP_RX_VALID | 1,
        USB_EP_TYPE_MASK | USB_EP_TX_VALID | USB_EP_RX_VALID | USB_EPADDR_FIELD
    );

    // Enable USB device and set address to 0.
    USB->DADDR = (USB->DADDR & ~(uint16_t)USB_DADDR_ADD) | USB_DADDR_EF;
}

static uint16_t get_rx_size(uint8_t ep_index) {
    return usb_btable[ep_index].count_rx & 0b0000001111111111;
}

static void send_request_error(void) {
    epnr_set(&USB->EP0R, USB_EP_TX_STALL, USB_EP_TX_VALID);
}

static void handle_clear_feature(const setup_packet* packet) {
    if (packet->wValue == 1) {
        // wValue of 1 corresponds to the DEVICE_REMOTE_WAKEUP feature.
        // This device does not support this feature, so just accept
        // this request, send the zero length status packet and do nothing.
        usb_btable[0].count_tx = 0;
        epnr_set(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
    }
    else {
        send_request_error();
    }
}

static void handle_get_configuration(const setup_packet* packet) {
    // Send one byte with the configuration state.
    copy_from_usb_sram(&configuration_state, ep0_tx_buffer, 1);
    usb_btable[0].count_tx = 1;
    epnr_set(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
}

static void handle_get_descriptor(const setup_packet* packet) {
    // Standard GET_DESCRIPTOR can only request device, configuration and string descriptors.
    const uint8_t descriptor_type = packet->wValue >> 8;
    const uint8_t descriptor_index = packet->wValue;
    const uint16_t descriptor_length = packet->wLength;

    if (descriptor_type == DEVICE) {
        uint16_t bytes_to_send = MIN(sizeof(DEVICE_DESCRIPTOR), descriptor_length);
        copy_to_usb_sram(ep0_tx_buffer, &DEVICE_DESCRIPTOR, bytes_to_send);
        usb_btable[0].count_tx = bytes_to_send;

        // Allow sending.
        epnr_set(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
    }
    else if (descriptor_type == CONFIGURATION && descriptor_index == 0) {
        uint16_t bytes_to_send = MIN(CONFIGURATION_DESCRIPTOR.wTotalLength, descriptor_length);
        uint16_t offset = 0;

        // Send the configuration descriptor, interface descriptor, hid descriptor and both endpoint
        // descriptors.
        copy_to_usb_sram(ep0_tx_buffer, &CONFIGURATION_DESCRIPTOR, sizeof(CONFIGURATION_DESCRIPTOR));
        offset += sizeof(CONFIGURATION_DESCRIPTOR);
        copy_to_usb_sram((volatile uint8_t*)ep0_tx_buffer + offset, &INTERFACE_DESCRIPTOR, sizeof(INTERFACE_DESCRIPTOR));
        offset += sizeof(INTERFACE_DESCRIPTOR);
        copy_to_usb_sram((volatile uint8_t*)ep0_tx_buffer + offset, &HID_DESCRIPTOR, sizeof(HID_DESCRIPTOR));
        offset += sizeof(HID_DESCRIPTOR);
        copy_to_usb_sram((volatile uint8_t*)ep0_tx_buffer + offset, &EP1_IN_DESCRIPTOR, sizeof(EP1_IN_DESCRIPTOR));
        offset += sizeof(EP1_IN_DESCRIPTOR);
        copy_to_usb_sram((volatile uint8_t*)ep0_tx_buffer + offset, &EP1_OUT_DESCRIPTOR, sizeof(EP1_OUT_DESCRIPTOR));

        usb_btable[0].count_tx = bytes_to_send;

        // Allow sending.
        epnr_set(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
    }
    else if (descriptor_type == STRING) {
        send_request_error();
    }
    else if (descriptor_type == DEVICE_QUALIFIER) {
        // This device does not support high-speed, so send the Request Error.
        send_request_error();
    }
    else if (descriptor_type == HID) {
        uint16_t bytes_to_send = MIN(sizeof(HID_DESCRIPTOR), descriptor_length);
        copy_to_usb_sram(ep0_tx_buffer, &HID_DESCRIPTOR, bytes_to_send);
        usb_btable[0].count_tx = bytes_to_send;

        // Allow sending.
        epnr_set(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
    }
    else if (descriptor_type == HID_REPORT) {
        uint16_t bytes_to_send = MIN(sizeof(HID_REPORT_DESCRIPTOR), descriptor_length);
        copy_to_usb_sram(ep0_tx_buffer, &HID_REPORT_DESCRIPTOR, bytes_to_send);
        usb_btable[0].count_tx = bytes_to_send;

        // Allow sending.
        epnr_set(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
    }
    else {
        // Throw the Request Error, if descriptor type or index is inappropriate.
        send_request_error();
    }
}

static void handle_get_interface(const setup_packet* packet) {
    // If the interface index is invalid, or the device is in the address state,
    // then send the Request Error.
    if (configuration_state == 0 || packet->wIndex != 0) {
        send_request_error();
        return;
    }

    // Send one zero byte, as the only interface we have is interface 0 and the only
    // alternate setting for this interface we have is 0.
    uint8_t tmp_zero = 0;
    copy_from_usb_sram(&tmp_zero, ep0_tx_buffer, 1);
    usb_btable[0].count_tx = 1;
    epnr_set(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
}

static void handle_get_status(const setup_packet* packet) {
    uint8_t recipient = packet->bmRequestType & 0b0001'1111;
    uint16_t data_to_send;

    if (recipient == 0) { // Device
        // The device is neither supports Remote Wakeup nor Self Powered.
        data_to_send = 0b0000'0000'0000'0000;
    }
    else if (recipient == 1) { // Interface
        // We have only 0th interface.
        if (packet->wIndex != 0 || configuration_state == 0) {
            send_request_error();
            return;
        }
        // All the bits are reserved.
        data_to_send = 0b0000'0000'0000'0000;
    }
    else if (recipient == 2) { // Endpoint
        // Only two endpoints are available.
        if (packet->wIndex > 1 || (configuration_state == 0 && packet->wIndex != 0)) {
            send_request_error();
            return;
        }
        // The halt feature is not supported for full-speed devices.
        data_to_send = 0b0000'0000'0000'0000;
    }
    else {
        send_request_error();
        return;
    }

    // Send these two bytes.
    copy_from_usb_sram(&data_to_send, ep0_tx_buffer, 2);
    usb_btable[0].count_tx = 2;
    epnr_set(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
}

static void handle_set_address(const setup_packet* packet) {
    // Send the zero length status packet.
    usb_btable[0].count_tx = 0;
    epnr_set(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
    // We will actually change our address later, after the successfull transfer
    // of the status packet.
}

static void handle_set_configuration(const setup_packet* packet) {
    uint8_t configuration_value = packet->wValue & 0x00FF;
    if (configuration_value == 0 || configuration_value == 1) {
        configuration_state = configuration_value;
        
        // Send the zero length status packet.
        usb_btable[0].count_tx = 0;
        epnr_set(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
    }
    else {
        send_request_error();
    }
}

static void handle_set_descriptor(const setup_packet* packet) {
    // This request is optinal and on this device is not supported.
    send_request_error();
}

static void handle_set_feature(const setup_packet* packet) {
    // This devices does not support neither DEVICE_REMOTE_WAKEUP,
    // nor ENDPOINT_HALT, nor TEST_MODE.
    send_request_error();
}

static void handle_set_interface(const setup_packet* packet) {
    // This device only has 0th interface and 0th alternate setting.
    if (packet->wIndex != 0 || packet->wValue != 0 || configuration_state == 0) {
        send_request_error();
        return;
    }

    // Send the zero length status packet.
    usb_btable[0].count_tx = 0;
    epnr_set(&USB->EP0R, USB_EP_TX_VALID, USB_EP_TX_VALID);
}

static void handle_synch_frame(const setup_packet* packet) {
    // This device does not operate in isochronous mode.
    send_request_error();
}

static void handle_setup_packet(void) {
    // The setup packet size is 8 bytes. If it is not, send STALL.
    if (get_rx_size(0) != 8) {
        send_request_error();
        return;
    }

    setup_packet packet;
    copy_from_usb_sram(&packet, ep0_rx_buffer, 8);
    control_state.packet = packet;

    switch (packet.bRequest) {
    case GET_STATUS: handle_get_status(&packet); break;
    case CLEAR_FEATURE: handle_clear_feature(&packet); break;
    case SET_FEATURE: handle_set_feature(&packet); break;
    case SET_ADDRESS: handle_set_address(&packet); break;
    case GET_DESCRIPTOR: handle_get_descriptor(&packet); break;
    case SET_DESCRIPTOR: handle_set_descriptor(&packet); break;
    case GET_CONFIGURATION: handle_get_configuration(&packet); break;
    case SET_CONFIGURATION: handle_set_configuration(&packet); break;
    case GET_INTERFACE: handle_get_interface(&packet); break;
    case SET_INTERFACE: handle_set_interface(&packet); break;
    case SYNCH_FRAME: handle_synch_frame(&packet); break;
    default: send_request_error(); break;
    }
}

static void handle_control(void) {
    if (USB->EP0R & USB_EP_CTR_RX) {
        // If the request is issued to the 0th endpoint, then there are two
        // possibilities:
        // 1. This is a setup packet with request like GET_STATUS or GET_DESCRIPTOR.
        // 2. This is a data packet which follows the SET_DESCRIPTOR and SYNCH_FRAME requests.
        //    We don't support neither of them here, normally it is impossible to receive
        //    a data packet like this. But if we did, ignore it.
        if (USB->EP0R & USB_EP_SETUP) {
            handle_setup_packet();
        }

        // Clear the CTR_RX bit and set STAT_RX to VALID.
        epnr_set(&USB->EP0R, USB_EP_RX_VALID, USB_EP_RX_VALID | USB_EP_CTR_RX);
    }
    else if (USB->EP0R & USB_EP_CTR_TX) {
        if (control_state.packet.bmRequestType == 0b00000000 && control_state.packet.bRequest == SET_ADDRESS) {
            // The SET_ADDRESS status packet was sent successfully.
            // Now we can change the address.
            USB->DADDR = (USB_DADDR & ~(uint16_t)USB_DADDR_ADD) | control_state.packet.wValue | USB_DADDR_EF;
        }

        // Clear the CTR_TX bit.
        epnr_set(&USB->EP0R, 0, USB_EP_CTR_TX);
    }
}

static void clear_cur_report(void) {
    for (uint8_t i = 0; i < HID_REPORT_SIZE; i++) {
        current_report[i] = 0;
    }
}

static void handle_endpoint_1(void) {
    if (USB->EP1R & USB_EP_CTR_RX) {
        // Ignore all the Interrupt Out and Feature requests.
        // Clear the CTR_RX bit and set STAT_RX to VALID.
        epnr_set(&USB->EP1R, USB_EP_RX_VALID, USB_EP_RX_VALID | USB_EP_CTR_RX);
    }
    else if (USB->EP1R & USB_EP_CTR_TX) {
        copy_to_usb_sram(ep1_tx_buffer, current_report, HID_REPORT_SIZE);
        usb_btable[1].count_tx = HID_REPORT_SIZE;
        // Allow transfer and clear the CTR_TX bit.
        epnr_set(&USB->EP1R, USB_EP_TX_VALID, USB_EP_TX_VALID | USB_EP_CTR_TX);
        
        // Keystroke has been sent. Unpress everything for the next request.
        clear_cur_report();
    }
}

void application_suspend(void) {

}

void application_wakeup(void) {

}

void __attribute__((interrupt("IRQ"))) usb_handler(void) {
    if (USB->ISTR & USB_ISTR_CTR) {
        USB->ISTR &= ~USB_ISTR_CTR;

        uint8_t endpoint = USB->ISTR & USB_ISTR_EP_ID;
        if (endpoint == 0) {
            handle_control();
        }
        else if (endpoint == 1) {
            handle_endpoint_1();
        }
    }

    if (USB->ISTR & USB_ISTR_ERR) {
        USB->ISTR &= ~USB_ISTR_ERR;
    }

    if (USB->ISTR & USB_ISTR_WKUP) {
        USB->ISTR &= ~USB_ISTR_WKUP;
        // Wakeup the USB peripheral and exit low power mode.
        USB->CNTR &= ~(USB_CNTR_FSUSP | USB_CNTR_LPMODE);
        application_wakeup();
    }

    if (USB->ISTR & USB_ISTR_SUSP) {
        USB->ISTR &= ~USB_ISTR_SUSP;
        // Suspend the USB peripheral and enter low power mode.
        USB->CNTR |= USB_CNTR_FSUSP | USB_CNTR_LPMODE;
        application_suspend();
    }

    if (USB->ISTR & USB_ISTR_RESET) {
        USB->ISTR &= ~USB_ISTR_RESET;
        handle_reset();
    }

    if (USB->ISTR & USB_ISTR_SOF) {
        USB->ISTR &= ~USB_ISTR_SOF;
    }
}

void usb_send_keystroke(enum key_modifier keymod, enum key key) {
    current_report[0] = keymod;
    current_report[1] = key;
}
