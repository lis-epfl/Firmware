/**
 * Enable Battery Failsafe System
 *
 * @boolean
 * @reboot_required true
 * @group Enable Battery Failsafe
 */
PARAM_DEFINE_INT32(BFS_ENABLED, 1);
/**
 * Enable Drone Configuration Based Battery Failsafe
 *
 * @reboot_required true
 * @group Battery Failsafe
 */
PARAM_DEFINE_INT32(BFS_ADV_FS, 1);
/**
 * Minimum thrust that must be available for one battery to fail
 *
 * @reboot_required true
 * @group Battery Failsafe
 * @min 0.2
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(BFS_ADV_THR_AVL, 0.5);
