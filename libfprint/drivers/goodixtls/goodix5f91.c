// Goodix TLS fingerprint driver for libfprint

// Copyright (C) 2026 custard <custard.py@gmail.com>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

#include "drivers/goodixtls/goodix5f91.h"
#include "drivers_api.h"
#include "fp-image-device.h"
#include "fpi-image-device.h"
#include "fpi-ssm.h"
#include "goodix.h"
#include "goodix5xx.h"
#include "goodix_proto.h"

#define FP_COMPONENT "goodixtls5f91"

typedef struct
{
  guint8 dummy;
} FpiDeviceGoodixTls5f91Private;

G_DECLARE_FINAL_TYPE (FpiDeviceGoodixTls5f91, fpi_device_goodixtls5f91, FPI,
                      DEVICE_GOODIXTLS5F91, FpiDeviceGoodixTls5xx)

struct _FpiDeviceGoodixTls5f91
{
  FpiDeviceGoodixTls5xx parent;
};

G_DEFINE_TYPE_WITH_PRIVATE (FpiDeviceGoodixTls5f91, fpi_device_goodixtls5f91,
                            FPI_TYPE_DEVICE_GOODIXTLS5XX)

enum activate_states {
  ACTIVATE_READ_AND_NOP,
  ACTIVATE_GET_FW,
  ACTIVATE_RESET,
  ACTIVATE_READ_REG,
  ACTIVATE_NOP2,
  ACTIVATE_READ_OTP,
  ACTIVATE_POV_CHECK,
  ACTIVATE_UPLOAD_CONFIG,
  ACTIVATE_SET_DRV_STATE,
  ACTIVATE_GET_POV_IMAGE,
  ACTIVATE_SWITCH_FDT_MODE,
  ACTIVATE_NUM_STATES,
};

enum scan_states {
  SCAN_QUERY_MCU,
  SCAN_GET_IMG,
  SCAN_DONE,
  SCAN_NUM_STATES,
};

static void
on_cmd_done (FpDevice *dev, guint8 *data, guint16 len, gpointer ssm,
             GError *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (ssm, error);
      return;
    }
  fpi_ssm_next_state (ssm);
}

static void
on_firmware (FpDevice *dev, gchar *firmware, gpointer ssm, GError *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (ssm, error);
      return;
    }
  fp_dbg ("5f91 firmware: %s", firmware);
  fpi_ssm_next_state (ssm);
}

static void
activate_run (FpiSsm *ssm, FpDevice *dev)
{
  switch (fpi_ssm_get_cur_state (ssm))
    {
    case ACTIVATE_READ_AND_NOP:
      goodix_start_read_loop (dev);
      goodix_send_nop (dev, goodixtls5xx_check_none, ssm);
      break;

    case ACTIVATE_GET_FW:
      goodix_send_query_firmware_version (dev, on_firmware, ssm);
      break;

    case ACTIVATE_RESET:
      goodix_send_protocol (dev, GOODIX_CMD_RESET, goodix_5f91_reset_payload,
                            sizeof (goodix_5f91_reset_payload), NULL, TRUE,
                            GOODIX_TIMEOUT, TRUE, on_cmd_done, ssm);
      break;

    case ACTIVATE_READ_REG:
      goodix_send_protocol (dev, GOODIX_CMD_READ_SENSOR_REGISTER,
                            goodix_5f91_read_reg_payload,
                            sizeof (goodix_5f91_read_reg_payload), NULL, TRUE,
                            GOODIX_TIMEOUT, TRUE, on_cmd_done, ssm);
      break;

    case ACTIVATE_NOP2:
      goodix_send_nop (dev, goodixtls5xx_check_none, ssm);
      break;

    case ACTIVATE_READ_OTP:
      goodix_send_protocol (dev, GOODIX_CMD_READ_OTP,
                            goodix_5f91_empty_payload,
                            sizeof (goodix_5f91_empty_payload), NULL, TRUE,
                            GOODIX_TIMEOUT, TRUE, on_cmd_done, ssm);
      break;

    case ACTIVATE_POV_CHECK:
      goodix_send_protocol (dev, 0xd6, goodix_5f91_pov_payload,
                            sizeof (goodix_5f91_pov_payload), NULL, TRUE,
                            GOODIX_TIMEOUT, TRUE, on_cmd_done, ssm);
      break;

    case ACTIVATE_UPLOAD_CONFIG:
      goodix_send_upload_config_mcu (dev, goodix_5f91_config,
                                     sizeof (goodix_5f91_config), NULL,
                                     goodixtls5xx_check_config_upload, ssm);
      break;

    case ACTIVATE_SET_DRV_STATE:
      goodix_send_protocol (dev, 0xc4, goodix_5f91_set_drv_state_payload,
                            sizeof (goodix_5f91_set_drv_state_payload), NULL,
                            TRUE, GOODIX_TIMEOUT, FALSE, on_cmd_done, ssm);
      break;

    case ACTIVATE_GET_POV_IMAGE:
      goodix_send_protocol (dev, 0xd2, goodix_5f91_empty_payload,
                            sizeof (goodix_5f91_empty_payload), NULL, TRUE,
                            GOODIX_TIMEOUT, TRUE, on_cmd_done, ssm);
      break;

    case ACTIVATE_SWITCH_FDT_MODE:
      goodix_send_mcu_switch_to_fdt_mode (dev, goodix_5f91_fdt_mode,
                                          sizeof (goodix_5f91_fdt_mode), NULL,
                                          on_cmd_done, ssm);
      break;
    }
}

static gboolean
activation_delayed_complete (gpointer user_data)
{
  fpi_image_device_activate_complete (FP_IMAGE_DEVICE (user_data), NULL);
  return G_SOURCE_REMOVE;
}

static void
tls_activation_complete (FpDevice *dev, gpointer user_data, GError *error)
{
  if (error)
    {
      fp_err ("5f91 TLS activation failed: %s", error->message);
      fpi_image_device_activate_complete (FP_IMAGE_DEVICE (dev), error);
      return;
    }
  /* The sensor needs a short settling delay after the TLS handshake;
   * otherwise it answers the first image command with a stray 0xd0 packet. */
  g_timeout_add (1000, activation_delayed_complete, dev);
}

static void
activate_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  if (error)
    {
      fp_err ("5f91 activation failed: %s", error->message);
      fpi_image_device_activate_complete (FP_IMAGE_DEVICE (dev), error);
      return;
    }
  goodix_tls_init (dev, tls_activation_complete, NULL);
}

static void
dev_activate (FpImageDevice *img_dev)
{
  fpi_ssm_start (fpi_ssm_new (FP_DEVICE (img_dev), activate_run,
                              ACTIVATE_NUM_STATES),
                 activate_complete);
}

static void
dev_init (FpImageDevice *img_dev)
{
  GError *error = NULL;

  if (goodix_dev_init (FP_DEVICE (img_dev), &error))
    {
      fpi_image_device_open_complete (img_dev, error);
      return;
    }
  fpi_image_device_open_complete (img_dev, NULL);
}

static void
dev_deinit (FpImageDevice *img_dev)
{
  GError *error = NULL;

  if (goodix_dev_deinit (FP_DEVICE (img_dev), &error))
    {
      fpi_image_device_close_complete (img_dev, error);
      return;
    }
  fpi_image_device_close_complete (img_dev, NULL);
}

static void
dev_deactivate (FpImageDevice *img_dev)
{
  GError *error = NULL;

  goodix_reset_state (FP_DEVICE (img_dev));
  goodix_shutdown_tls (FP_DEVICE (img_dev), &error);
  fpi_image_device_deactivate_complete (img_dev, error);
}

/* Decode 12-bit packed frame and orient it the same way as the PoC:
   final[y][x] = unpacked[x * 56 + (55 - y)].
   The result is histogram-equalized, inverted (dark ridges on light
   background) and upscaled 4x so NBIS can extract useful minutiae. */
static FpImage *
make_image (const guint8 *raw, guint16 len)
{
  guint32 pixels = GOODIX_5F91_WIDTH * GOODIX_5F91_HEIGHT;
  guint32 n = len / 6;

  if (n > pixels / 4)
    n = pixels / 4;

  guint16 *unpacked = g_malloc0 (pixels * sizeof (guint16));
  for (guint32 i = 0; i < n; i++)
    {
      const guint8 *c = raw + i * 6;
      unpacked[i * 4 + 0] = ((c[0] & 0xf) << 8) + c[1];
      unpacked[i * 4 + 1] = (c[3] << 4) + (c[0] >> 4);
      unpacked[i * 4 + 2] = ((c[5] & 0xf) << 8) + c[2];
      unpacked[i * 4 + 3] = (c[4] << 4) + (c[5] >> 4);
    }

  /* Histogram equalization over the native 176x56 image. */
  guint32 hist[4096] = {0};
  for (guint32 i = 0; i < pixels; i++)
    {
      guint16 v = unpacked[i];
      hist[v]++;
    }
  guint32 cdf[4096];
  guint32 sum = 0;
  for (guint32 i = 0; i < 4096; i++)
    {
      sum += hist[i];
      cdf[i] = sum * 255 / pixels;
    }

  guint8 *base = g_malloc (pixels);
  for (guint y = 0; y < GOODIX_5F91_HEIGHT; y++)
    {
      for (guint x = 0; x < GOODIX_5F91_WIDTH; x++)
        {
          guint32 idx = x * 56 + (55 - y);
          guint16 v = idx < pixels ? unpacked[idx] : 0;
          base[x + y * GOODIX_5F91_WIDTH] = 255 - (guchar) cdf[v];
        }
    }
  g_free (unpacked);

  FpImage *img = fp_image_new (GOODIX_5F91_OUT_WIDTH,
                               GOODIX_5F91_OUT_HEIGHT);
  for (guint y = 0; y < GOODIX_5F91_OUT_HEIGHT; y++)
    {
      for (guint x = 0; x < GOODIX_5F91_OUT_WIDTH; x++)
        {
          guint sx = x / GOODIX_5F91_SCALE;
          guint sy = y / GOODIX_5F91_SCALE;
          img->data[x + y * GOODIX_5F91_OUT_WIDTH] =
            base[sx + sy * GOODIX_5F91_WIDTH];
        }
    }
  g_free (base);
  return img;
}

static void
scan_on_image (FpDevice *dev, guint8 *data, guint16 len, gpointer ssm,
               GError *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (ssm, error);
      return;
    }
  if (len < 8)
    {
      fpi_ssm_mark_failed (ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "short image"));
      return;
    }
  FpImage *img = make_image (data, len - 4);
  fpi_image_device_image_captured (FP_IMAGE_DEVICE (dev), img);
  fpi_ssm_next_state (ssm);
}

static void
scan_query_cb (FpDevice *dev, guint8 *data, guint16 len, gpointer ssm,
               GError *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (ssm, error);
      return;
    }

  /* This firmware always replies 01 00 to QUERY_MCU_STATE, regardless of
  * finger presence, and NAV_0 gets no reply. Capture unconditionally. */
  fpi_ssm_next_state (ssm);
}

static void
scan_run (FpiSsm *ssm, FpDevice *dev)
{
  switch (fpi_ssm_get_cur_state (ssm))
    {
    case SCAN_QUERY_MCU:
      goodix_send_query_mcu_state (dev, scan_query_cb, ssm);
      break;

    case SCAN_GET_IMG:
      fpi_image_device_report_finger_status (FP_IMAGE_DEVICE (dev), TRUE);
      goodix_tls_read_image (dev, scan_on_image, ssm);
      break;

    case SCAN_DONE:
      fpi_image_device_report_finger_status (FP_IMAGE_DEVICE (dev), FALSE);
      fpi_ssm_next_state (ssm);
      break;
    }
}

static void
scan_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  if (error)
    {
      fp_err ("5f91 scan failed: %s", error->message);
      fpi_image_device_session_error (FP_IMAGE_DEVICE (dev), error);
    }
}

static void
dev_change_state (FpImageDevice *img_dev, FpiImageDeviceState state)
{
  if (state == FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON)
    fpi_ssm_start (fpi_ssm_new (FP_DEVICE (img_dev), scan_run, SCAN_NUM_STATES),
                   scan_complete);
}

static void
fpi_device_goodixtls5f91_init (FpiDeviceGoodixTls5f91 *self)
{
}

static void
fpi_device_goodixtls5f91_class_init (FpiDeviceGoodixTls5f91Class *class)
{
  FpiDeviceGoodixTlsClass *tls_class = FPI_DEVICE_GOODIXTLS_CLASS (class);
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (class);
  FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS (class);

  tls_class->interface = GOODIX_5F91_INTERFACE;
  tls_class->ep_in = GOODIX_5F91_EP_IN;
  tls_class->ep_out = GOODIX_5F91_EP_OUT;
  tls_class->skip_tls_success = TRUE;
  tls_class->image_flags = GOODIX_5F91_IMAGE_FLAGS;
  tls_class->image_prefix_len = GOODIX_5F91_IMAGE_PREFIX_LEN;

  dev_class->id = "goodixtls5f91";
  dev_class->full_name = "Goodix TLS Fingerprint Sensor 5F91";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->nr_enroll_stages = 20;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;

  img_class->bz3_threshold = 24;
  img_class->img_width = GOODIX_5F91_OUT_WIDTH;
  img_class->img_height = GOODIX_5F91_OUT_HEIGHT;
  img_class->activate = dev_activate;
  img_class->deactivate = dev_deactivate;
  img_class->img_open = dev_init;
  img_class->img_close = dev_deinit;
  img_class->change_state = dev_change_state;

  fpi_device_class_auto_initialize_features (dev_class);
}
