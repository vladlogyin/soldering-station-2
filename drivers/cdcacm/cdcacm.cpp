#include <cdcacm/cdcacm.h>


static enum usbd_request_return_codes cdcacm_control_request ( usbd_device* usbd_dev,
        struct usb_setup_data* req,
        uint8_t** buf,
        uint16_t* len,
        void ( **complete ) ( usbd_device* usbd_dev,
                              struct usb_setup_data* req ) )
{
    ( void ) complete;
    ( void ) buf;
    ( void ) usbd_dev;

    switch ( req->bRequest ) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
        /*
         * This Linux cdc_acm driver requires this to be implemented
         * even though it's optional in the CDC spec, and we don't
         * advertise it in the ACM functional descriptor.
         */
        char local_buf[10];
        struct usb_cdc_notification* notif = ( usb_cdc_notification* ) local_buf;

        /* We echo signals back to host as notification. */
        notif->bmRequestType = 0xA1;
        notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
        notif->wValue = 0;
        notif->wIndex = 0;
        notif->wLength = 2;
        local_buf[8] = req->wValue & 3;
        local_buf[9] = 0;
        // usbd_ep_write_packet(0x83, buf, 10);
        return USBD_REQ_HANDLED;
    }
    case USB_CDC_REQ_SET_LINE_CODING:
        if ( *len < sizeof ( struct usb_cdc_line_coding ) ) {
            return USBD_REQ_NOTSUPP;
        }
        return USBD_REQ_HANDLED;
    }
    return USBD_REQ_NOTSUPP;
}

void cdcacm_data_rx_cb ( usbd_device* usbd_dev, uint8_t ep )
{
    ( void ) ep;
    char buf[64];
    int len = usbd_ep_read_packet ( usbd_dev, 0x01, buf, 64 );

}

void cdcacm_set_config ( usbd_device* usbd_dev, uint16_t wValue )
{
    ( void ) wValue;

    usbd_ep_setup ( usbd_dev, 0x01, USB_ENDPOINT_ATTR_INTERRUPT, 64, cdcacm_data_rx_cb );
    usbd_ep_setup ( usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, nullptr );
    usbd_ep_setup ( usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, nullptr );

    usbd_register_control_callback ( usbd_dev,
                                     USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
                                     USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                                     cdcacm_control_request );
}
