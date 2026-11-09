#ifndef DIRECTION_CONFIG_H_
#define DIRECTION_CONFIG_H_

// 2026-09-21 已确认实车基准。菜单 +1=保持此基准，-1=相对反向。
// 这些符号属于硬件/控制适配，不能再次当作用户默认值写入参数。
// 与 PARAM_MAGIC 的 sign2 语义绑定；未来改基准须新增存储版本和迁移。
#define MOTOR_DIR_A_BASE      (1)
#define MOTOR_DIR_B_BASE      (1)
/* Raw encoder RPM polarity relative to a positive DRIVER command, not body Roll.
 * +1: positive driver duty reports positive raw RPM; -1: reports negative RPM.
 * Defaults preserve the old receive polarity. Verify actual CYT calibration
 * on the board. These are DEFAULTS for Motor -> Enc A/B when Flash has no key;
 * saved enc_dir_a/b override them independently of MOTOR_DIR. */
#ifndef FLY_ENCODER_POLARITY_A
#define FLY_ENCODER_POLARITY_A (1)
#endif
#ifndef FLY_ENCODER_POLARITY_B
#define FLY_ENCODER_POLARITY_B (1)
#endif
#if (FLY_ENCODER_POLARITY_A != 1 && FLY_ENCODER_POLARITY_A != -1) || \
    (FLY_ENCODER_POLARITY_B != 1 && FLY_ENCODER_POLARITY_B != -1)
#error "Flywheel encoder polarity must be +1 or -1"
#endif
#define MOTOR_DIR_C_BASE      (-1)
#define ENC_DIR_C_BASE        (-1)
#define STEER_DIR_BASE        (-1)
#define IMU_FORWARD_DIR_BASE  (-1)

// R_RATE_KD_BASE is retained ONLY for legacy Flash migration. The current
// Roll Rate controller is incremental PI and never uses a derivative gain.
#define R_RATE_KD_BASE        (-1.0f)
#define Y_RATE_BASE           (-1.0f)

#endif
