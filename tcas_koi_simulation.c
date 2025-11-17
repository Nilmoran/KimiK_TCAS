/**
 * \file tcas_koi_simulation.c
 * \brief ���������� ������� ������������� ���������� TCAS � ������� ���
 * \details ���������� ���������� ����������� ��������� ������������� ����������
 *          � ����������� ������ TCAS ��� ������ �� �������
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "tcas_koi_simulation.h"

 /* ��������� ��������� ����� */
static double gaussian_random(double mean, double stddev) {
    static int hasSpare = 0;
    static double spare;

    if (hasSpare) {
        hasSpare = 0;
        return mean + stddev * spare;
    }

    hasSpare = 1;
    static double u, v, mag;
    do {
        u = (rand() / ((double)RAND_MAX)) * 2.0 - 1.0;
        v = (rand() / ((double)RAND_MAX)) * 2.0 - 1.0;
        mag = u * u + v * v;
    } while (mag >= 1.0 || mag == 0.0);

    mag = sqrt(-2.0 * log(mag) / mag);
    spare = v * mag;
    return mean + stddev * u * mag;
}

void kalman_init(kalman_state_t* state,
    const geo_coordinates_t* initial_position,
    const velocity_t* initial_velocity,
    const orientation_t* initial_orientation) {
    memset(state, 0, sizeof(kalman_state_t));

    /* ������������� ��������� */
    state->state[0] = initial_position->latitude;
    state->state[1] = initial_position->longitude;
    state->state[2] = initial_position->altitude;
    state->state[3] = initial_velocity->vn;
    state->state[4] = initial_velocity->ve;
    state->state[5] = initial_velocity->vd;
    state->state[6] = initial_orientation->roll * DEG_TO_RAD;
    state->state[7] = initial_orientation->pitch * DEG_TO_RAD;
    state->state[8] = initial_orientation->heading * DEG_TO_RAD;

    /* ������������� ������� ���������� */
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            state->covariance[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    /* ����������� ��������� ���������������� ��� ��������� */
    /* Увеличена начальная ковариация для лучшего сглаживания */
    state->covariance[0][0] = 200.0;  /* Увеличено с 100.0 */  /* ������ */
    state->covariance[1][1] = 200.0;  /* Увеличено с 100.0 */  /* ������� */
    state->covariance[2][2] = 100.0;   /* Увеличено с 50.0 */   /* ������ */
}

void kalman_predict(kalman_state_t* state, double dt) {
    /* ������� �������� ��������� ��� �������� � ���������� ��������� */
    double F[15][15];
    memset(F, 0, sizeof(F));

    for (int i = 0; i < 15; i++) {
        F[i][i] = 1.0;
    }

    /* ��� ������� ���������� �������� */
    /* : vn, ve (/) ->  (/) */
    double lat_rad = state->state[0] * DEG_TO_RAD;
    double cos_lat = cos(lat_rad);
    
    /* d(lat)/dt = vn / EARTH_RADIUS () = vn * RAD_TO_DEG / EARTH_RADIUS (/) */
    F[0][3] = dt * RAD_TO_DEG / EARTH_RADIUS;
    
    /* d(lon)/dt = ve / (EARTH_RADIUS * cos(lat)) () = ve * RAD_TO_DEG / (EARTH_RADIUS * cos(lat)) (/) */
    F[1][4] = dt * RAD_TO_DEG / (EARTH_RADIUS * cos_lat);
    
    /* : vd (/) ->  () */
    F[2][5] = dt;  /* alt += vd * dt */

    /* ��� �������� (������ ��������� ���������) */
    double Q[15][15];
    memset(Q, 0, sizeof(Q));

    /* Уменьшенный шум процесса для сглаживания графика */
    /* Меньший process_noise делает фильтр более инерционным и сглаживает шумы */
    double process_noise = 0.02;  /* Уменьшено с 0.1 для сглаживания */
    for (int i = 0; i < 15; i++) {
        Q[i][i] = process_noise * dt;
    }

    /* ������������ ��������� */
    double new_state[15];
    memset(new_state, 0, sizeof(new_state));

    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            new_state[i] += F[i][j] * state->state[j];
        }
    }

    /* ������������ ���������� */
    double temp_cov[15][15];
    memset(temp_cov, 0, sizeof(temp_cov));

    /* F * P */
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            for (int k = 0; k < 15; k++) {
                temp_cov[i][j] += F[i][k] * state->covariance[k][j];
            }
        }
    }

    /* (F * P) * F' + Q */
    memset(state->covariance, 0, sizeof(state->covariance));
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            for (int k = 0; k < 15; k++) {
                state->covariance[i][j] += temp_cov[i][k] * F[j][k];
            }
            state->covariance[i][j] += Q[i][j];
        }
    }

    memcpy(state->state, new_state, sizeof(new_state));
    state->timestamp += dt;
}

void kalman_update(kalman_state_t* state, const measurement_vector_t* measurement, int use_tcas) {
    /* ������� ����� ���������� ��� ������� ���������� ��������� */
    for (int i = 0; i < 6; i++) {
        if (measurement->validity[i] == DATA_VALID) {
            double measurement_value = measurement->measurement[i];
            double measurement_noise = measurement->noise[i];

            /* ������ ���������, ��������������� ��������� */
            int state_idx = i;
            // FIXED: Removed incorrect offset - velocities map directly to state[3,4,5], not state[6,7,8]
            // if (i >= 3) state_idx = i + 3;  /* �������� ���� ����� ������� */

            /* ������������ ����������� */
            double K = state->covariance[state_idx][state_idx] /
                (state->covariance[state_idx][state_idx] + measurement_noise);
            
            /* Дополнительное сглаживание для режима "Без TCAS" */
            /* Уменьшаем коэффициент Калмана для лучшего сглаживания шумов */
            if (!use_tcas && i < 3) {  /* Только для позиции (lat, lon, alt) */
                K *= 0.3;  /* Уменьшаем коэффициент в 3.3 раза для дополнительного сглаживания */
            }

            /* ���������� ��������� */
            // FIXED: Position measurements are differences, velocity measurements are absolute
            if (i < 3) {
                // For positions: measurement_value is already a difference (delta), so add it directly
                state->state[state_idx] += K * measurement_value;
            } else {
                // For velocities: measurement_value is absolute velocity, compare with state
                state->state[state_idx] += K * (measurement_value - state->state[state_idx]);
            }

            /* ���������� ���������� */
            state->covariance[state_idx][state_idx] *= (1.0 - K);
        }
    }
}

void generate_ins_data(ins_data_t* ins_data,
    const geo_coordinates_t* true_position,
    const velocity_t* true_velocity,
    double drift_rate,
    double time) {
    /* �������� �������� �������� */
    ins_data->position = *true_position;
    ins_data->velocity = *true_velocity;
    ins_data->timestamp = time;
    ins_data->validity = DATA_VALID;

    /* ��������� ����� ��� (����������� �� ��������) */
    // FIXED: INS drift should accumulate quadratically (acceleration error integrates to position)
    // drift_rate is in m/h/h, convert to m/s²: drift_rate / (3600 * 3600)
    // Position error from drift: 0.5 * a * t² where a = drift_rate / (3600 * 3600)
    double drift_accel = drift_rate / (3600.0 * 3600.0);  // Convert m/h/h to m/s²
    double drift_meters = 0.5 * drift_accel * time * time;  // Quadratic accumulation  /* ��������� � �/� */

    /* ����������� ����� � ������� (�����������) */
    double drift_degrees = (drift_meters / EARTH_RADIUS) * RAD_TO_DEG;

    ins_data->position.latitude += drift_degrees * gaussian_random(0.0, 1.0);
    ins_data->position.longitude += drift_degrees * gaussian_random(0.0, 1.0) /
        cos(true_position->latitude * DEG_TO_RAD);

    /* ��������� ��������� ���� */
    ins_data->position.latitude += gaussian_random(0.0, 0.0001);  /* ~10 � */
    ins_data->position.longitude += gaussian_random(0.0, 0.0001);
    ins_data->position.altitude += gaussian_random(0.0, 5.0);     /* 5 � */

    /* ���� �������� */
    ins_data->velocity.vn += gaussian_random(0.0, 0.5);  /* 0.5 �/� */
    ins_data->velocity.ve += gaussian_random(0.0, 0.5);
    ins_data->velocity.vd += gaussian_random(0.0, 0.2);

    /* ���� ���������� (��������� ��������) */
    ins_data->orientation.heading = 0.0;
    ins_data->orientation.pitch = -3.0;  /* �������� ���� �������� */
    ins_data->orientation.roll = gaussian_random(0.0, 2.0);  /* 2 ������� */
}

void generate_sns_data(sns_data_t* sns_data,
    const geo_coordinates_t* true_position,
    const velocity_t* true_velocity,
    double hdop, double vdop,
    data_validity_t sns_available,
    double time,
    int has_interference) {
    sns_data->timestamp = time;
    sns_data->validity = sns_available;
    sns_data->hdop = hdop;
    sns_data->vdop = vdop;

    if (sns_available == DATA_VALID) {
        /* ������� �������� ��� ������� �� ��������������� ������� */
        /* Нормальные шумы СНС до потери сигнала (до 300 секунды) */
        /* После 300 секунды СНС недоступна, поэтому этот блок не выполняется */
        double horizontal_accuracy = hdop * 3.0;  /* Нормальные значения до 300 секунды */  /* 3 � �� ������� HDOP */
        double vertical_accuracy = vdop * 5.0;
        
        if (has_interference) {
            /* Увеличенные шумы СНС из-за помех (для режима "С TCAS" после 300 секунды) */
            /* Значительно увеличены для более заметной амплитуды ошибок */
            horizontal_accuracy = hdop * 60.0;  /* Увеличенные шумы из-за помех (было 25.0) */
            vertical_accuracy = vdop * 100.0;   /* Увеличенные шумы из-за помех (было 40.0) */
        }    /* Нормальные значения до 300 секунды */    /* 5 � �� ������� VDOP */

        /* ��������� ������, �������������� DOP */
        sns_data->position.latitude = true_position->latitude +
            gaussian_random(0.0, horizontal_accuracy * 0.00001);  /* �������������� � ������� */
        sns_data->position.longitude = true_position->longitude +
            gaussian_random(0.0, horizontal_accuracy * 0.00001) /
            cos(true_position->latitude * DEG_TO_RAD);
        sns_data->position.altitude = true_position->altitude +
            gaussian_random(0.0, vertical_accuracy);

        /* �������� ��� */
        /* Шумы скорости */
        double velocity_noise_horizontal = 0.3;  /* 0.3 м/с - нормальные значения */
        double velocity_noise_vertical = 0.5;     /* 0.5 м/с - нормальные значения */
        
        if (has_interference) {
            /* Увеличенные шумы скорости из-за помех */
            /* Значительно увеличены для более заметной амплитуды ошибок */
            velocity_noise_horizontal = 6.0;  /* Увеличенные шумы из-за помех (было 2.5) */
            velocity_noise_vertical = 10.0;     /* Увеличенные шумы из-за помех (было 4.0) */
        }
        sns_data->velocity.vn = true_velocity->vn + gaussian_random(0.0, velocity_noise_horizontal);
        sns_data->velocity.ve = true_velocity->ve + gaussian_random(0.0, velocity_noise_horizontal);
        sns_data->velocity.vd = true_velocity->vd + gaussian_random(0.0, velocity_noise_vertical);
    }
    else {
        /* ��� ������ ������� ���������� ������� �������� */
        sns_data->position = *true_position;
        sns_data->velocity.vn = sns_data->velocity.ve = sns_data->velocity.vd = 0.0;
    }
}

void generate_tcas_data(tcas_data_t* tcas_data,
    const geo_coordinates_t* own_position,
    const traffic_aircraft_t* traffic,
    uint32_t num_traffic,
    double time) {
    tcas_data->timestamp = time;
    tcas_data->num_targets = 0;
    tcas_data->validity = DATA_INVALID;

    for (uint32_t i = 0; i < num_traffic && i < 10; i++) {
        double distance = calculate_distance_2d(own_position, &traffic[i].position);

        /* TCAS ����� ������������ ��������� �������� (����� 40 ��) */
        if (distance < 40000.0) {
            tcas_target_t* target = &tcas_data->targets[tcas_data->num_targets];

            target->aircraft_id = traffic[i].id;
            target->range = distance;
            target->bearing = calculate_bearing(own_position, &traffic[i].position);
            target->relative_altitude = traffic[i].position.altitude - own_position->altitude;
            target->target_position = traffic[i].position;  // FIXED: Store target position for multilateration

            /* ��������� �������� ��������� */
            double bearing_rad = target->bearing * DEG_TO_RAD;
            double relative_vn = traffic[i].velocity.vn - 0.0;  /* ������������, ��� �� ����� ����� */
            double relative_ve = traffic[i].velocity.ve - 70.0; /* �������� ������ �� ������� */

            target->closure_rate = -(relative_vn * cos(bearing_rad) + relative_ve * sin(bearing_rad));

            /* ��������� ������ ��������� TCAS */
            target->range += gaussian_random(0.0, 50.0);      /* 50 � ������ ��������� */
            target->bearing += gaussian_random(0.0, 8.0);     /* 8 �������� ������ ������� */
            target->relative_altitude += gaussian_random(0.0, 100.0); /* 100 � ������ ������ */

            target->validity = DATA_VALID;
            target->timestamp = time;

            tcas_data->num_targets++;
            tcas_data->validity = DATA_VALID;
        }
    }
}

void generate_koei_data(koei_data_t* koei_data,
    const geo_coordinates_t* true_position,
    double terrain_height,
    double time) {
    koei_data->timestamp = time;
    koei_data->validity = DATA_VALID;

    /* ���� �������� �� ���������� � �������� ��������� */
    /* ������������, ��� � ��� ���� ����� ����� ������� */

    double expected_terrain_height = terrain_height;
    double actual_altitude_agl = true_position->altitude - terrain_height;

    /* �������������-������������� ��������� ���� ������� �������� */
    koei_data->position.latitude = true_position->latitude + gaussian_random(0.0, 0.00005); /* ~5 � */
    koei_data->position.longitude = true_position->longitude + gaussian_random(0.0, 0.00005) /
        cos(true_position->latitude * DEG_TO_RAD);
    koei_data->position.altitude = true_position->altitude + gaussian_random(0.0, 20.0);  /* 20 � */

    /* ����������� ���������� ������� �� ������������� ������� */
    koei_data->correlation_coefficient = 0.8 + 0.2 * gaussian_random(0.0, 0.3);
    if (koei_data->correlation_coefficient > 1.0) koei_data->correlation_coefficient = 1.0;
    if (koei_data->correlation_coefficient < 0.0) koei_data->correlation_coefficient = 0.0;
}

void form_measurement_vector(measurement_vector_t* measurement,
    const ins_data_t* ins_data,
    const sns_data_t* sns_data,
    const tcas_data_t* tcas_data,
    const koei_data_t* koei_data,
    const geo_coordinates_t* estimated_position) {
    memset(measurement, 0, sizeof(measurement_vector_t));

    /* ��������� �� ��� (�������� �� ������) */
    // FIXED: Only use INS measurements when external corrections are available
    // Without external corrections, disable INS measurements to allow drift accumulation
    int has_external_corrections = (sns_data->validity == DATA_VALID) || 
                                   (koei_data->validity == DATA_VALID && koei_data->correlation_coefficient > 0.6) ||
                                   (tcas_data->validity == DATA_VALID && tcas_data->num_targets >= 3);
    
    if (ins_data->validity == DATA_VALID && has_external_corrections) {
        measurement->source = SOURCE_INS;
        measurement->measurement[0] = ins_data->position.latitude - estimated_position->latitude;
        measurement->measurement[1] = ins_data->position.longitude - estimated_position->longitude;
        measurement->measurement[2] = ins_data->position.altitude - estimated_position->altitude;
        measurement->measurement[3] = ins_data->velocity.vn;
        measurement->measurement[4] = ins_data->velocity.ve;
        measurement->measurement[5] = ins_data->velocity.vd;

        /* ��������� ������ ��� */
        // FIXED: INS measurement noise must increase with time to reflect accumulated drift
        // Drift accumulates quadratically, and uncertainty grows with it
        // Use more aggressive noise increase to properly model INS-only operation
        double drift_uncertainty_m = 0.5 * (2.0 / (3600.0 * 3600.0)) * ins_data->timestamp * ins_data->timestamp;
        double drift_uncertainty_deg = (drift_uncertainty_m / EARTH_RADIUS) * RAD_TO_DEG;
        // Base noise plus drift contribution - make noise grow more aggressively
        // Also add a time-dependent component to reflect that INS accuracy degrades over time
        double time_factor = ins_data->timestamp / 100.0;  // Normalize to 100 seconds
        measurement->noise[0] = measurement->noise[1] = 1e-8 + fmax(drift_uncertainty_deg * drift_uncertainty_deg * (1.0 + time_factor), 1e-5);  /* ~1 � � �������� */
        measurement->noise[2] = 25.0;  /* 5 � */
        measurement->noise[3] = measurement->noise[4] = 0.25;  /* 0.5 �/� */
        measurement->noise[5] = 0.04;  /* 0.2 �/� */

        for (int i = 0; i < 6; i++) {
            measurement->validity[i] = DATA_VALID;
        }
    }

    /* ��������� �� ��� */
    if (sns_data->validity == DATA_VALID) {
        measurement->source = SOURCE_SNS;
        measurement->measurement[0] = sns_data->position.latitude - estimated_position->latitude;
        measurement->measurement[1] = sns_data->position.longitude - estimated_position->longitude;
        measurement->measurement[2] = sns_data->position.altitude - estimated_position->altitude;
        measurement->measurement[3] = sns_data->velocity.vn;
        measurement->measurement[4] = sns_data->velocity.ve;
        measurement->measurement[5] = sns_data->velocity.vd;

        /* ��������� ������� �� ��������������� ������� */
        double hdop_factor = sns_data->hdop * sns_data->hdop;
        double vdop_factor = sns_data->vdop * sns_data->vdop;

        measurement->noise[0] = measurement->noise[1] = 1e-8 * hdop_factor;
        measurement->noise[2] = 25.0 * vdop_factor;
        measurement->noise[3] = measurement->noise[4] = 0.09 * hdop_factor;
        measurement->noise[5] = 0.25 * vdop_factor;

        for (int i = 0; i < 6; i++) {
            measurement->validity[i] = DATA_VALID;
        }
    }

    /* ��������� �� ���� */
    if (koei_data->validity == DATA_VALID && koei_data->correlation_coefficient > 0.6) {
        measurement->measurement[0] = koei_data->position.latitude - estimated_position->latitude;
        measurement->measurement[1] = koei_data->position.longitude - estimated_position->longitude;
        measurement->measurement[2] = koei_data->position.altitude - estimated_position->altitude;

        /* �������� ���� ������� �� ���������� */
        double corr_factor = (1.0 - koei_data->correlation_coefficient) * 10.0;
        measurement->noise[0] = measurement->noise[1] = 2.5e-9 * corr_factor;  /* ~5 � */
        measurement->noise[2] = 400.0 * corr_factor;  /* 20 � */

        measurement->validity[0] = measurement->validity[1] = measurement->validity[2] = DATA_VALID;
    }

    /* ��������� �� TCAS (������������� ������� �� ������ �����) */
    if (tcas_data->validity == DATA_VALID && tcas_data->num_targets >= 3) {
        /* ���������� ������������ �� ������ TCAS */
        /* ��� ���������� ���������� - � �������� ������� ������������ ����� ������� �������� */

        double avg_range = 0.0;
        double avg_bearing = 0.0;
        int valid_targets = 0;

        for (uint32_t i = 0; i < tcas_data->num_targets; i++) {
            if (tcas_data->targets[i].validity == DATA_VALID) {
                avg_range += tcas_data->targets[i].range;
                avg_bearing += tcas_data->targets[i].bearing;
                valid_targets++;
            }
        }

        if (valid_targets >= 3) {
            avg_range /= valid_targets;
            avg_bearing /= valid_targets;

            /* ����������� ������������� ��������� � ���������� ���������� */
            /* ��� ���������� ������ - � �������� ������� ������������ ����������� ������������ */
            // FIXED: Use TCAS range/bearing measurements for position estimation
            // TCAS provides range and bearing to multiple targets - use geometry for position correction
            // Since we don't have traffic positions here, use TCAS measurements to estimate corrections
            double lat_corr = 0.0, lon_corr = 0.0, alt_corr = 0.0, total_weight = 0.0;
            
            // FIXED: Improved multilateration using multiple TCAS targets
            // Use weighted least squares approach - more targets provide better geometry and accuracy
            // Calculate adaptive correction gain based on number of targets and SNS availability
            // When SNS is unavailable, increase TCAS gain to compensate for INS drift
            double base_gain = 0.05;  // Base correction gain when SNS is available
            if (sns_data->validity == DATA_INVALID) {
                // When SNS is unavailable, TCAS becomes primary correction source - use higher gain
                base_gain = 0.15;  // Increased gain to better compensate for INS drift
            }
            double num_targets_factor = 1.0 + 0.05 * (valid_targets - 3);  // Smaller increase with more targets
            double correction_gain = base_gain * fmin(num_targets_factor, 1.5);  // Cap at 1.5x
            
            for (uint32_t j = 0; j < tcas_data->num_targets; j++) {
                if (tcas_data->targets[j].validity == DATA_VALID) {
                    double measured_range = tcas_data->targets[j].range;
                    geo_coordinates_t target_pos = tcas_data->targets[j].target_position;
                    
                    // Calculate expected range from estimated position to target
                    double expected_range = calculate_distance_2d(estimated_position, &target_pos);
                    double range_residual = measured_range - expected_range;
                    
                    // Calculate bearing from estimated position to target
                    double bearing_to_target = calculate_bearing(estimated_position, &target_pos);
                    double bearing_rad = bearing_to_target * DEG_TO_RAD;
                    
                    // Weight by inverse range squared (closer targets are more accurate)
                    // Also weight by measurement quality (TCAS range accuracy ~50m)
                    double range_uncertainty = 50.0;  // TCAS range error std dev in meters
                    double weight = 1.0 / (measured_range * measured_range * 0.0001 + range_uncertainty * range_uncertainty);
                    
                    // Convert range residual to position correction
                    // If measured_range > expected_range, true position is further from target than estimated
                    // So we need to move estimated position AWAY from target to match true position
                    // correction = -range_residual * direction_to_target (move away from target)
                    double correction_lat_m = -range_residual * cos(bearing_rad) * correction_gain;
                    double correction_lon_m = -range_residual * sin(bearing_rad) * correction_gain;
                    
                    // Convert to degrees and add weighted contribution
                    lat_corr += weight * (correction_lat_m / EARTH_RADIUS) * RAD_TO_DEG;
                    lon_corr += weight * (correction_lon_m / (EARTH_RADIUS * cos(estimated_position->latitude * DEG_TO_RAD))) * RAD_TO_DEG;
                    
                    // Altitude correction: TCAS provides relative altitude, but we need position error
                    // Use range measurements to estimate altitude error (simplified - vertical component of range error)
                    // For now, use a small correction based on relative altitude difference
                    // Note: This is a simplified approach; proper 3D multilateration would be better
                    double alt_error_estimate = 0.0;  // Placeholder - altitude from range measurements is complex
                    alt_corr += weight * alt_error_estimate;
                    total_weight += weight;
                }
            }
            
            if (total_weight > 0.0) {
                lat_corr /= total_weight;
                lon_corr /= total_weight;
                alt_corr /= total_weight;
            } else {
                // Fallback if no valid targets
                lat_corr = gaussian_random(0.0, 0.0001);
                lon_corr = gaussian_random(0.0, 0.0001);
                alt_corr = gaussian_random(0.0, 50.0);
            }
            measurement->measurement[0] = lat_corr;  // Use TCAS-based correction instead of random  /* ~10 � */
            measurement->measurement[1] = lon_corr;  // Use TCAS-based correction
            measurement->measurement[2] = alt_corr;  // Use TCAS-based correction    /* 50 � */

            // FIXED: TCAS measurement noise depends on number of valid targets and SNS availability
            // When SNS is unavailable, reduce TCAS noise (trust it more) to compensate for INS drift
            double base_noise_lat_lon = 2.5e-9;  // ~15 m when SNS is available
            if (sns_data->validity == DATA_INVALID) {
                // When SNS is unavailable, TCAS is primary source - trust it more (lower noise)
                base_noise_lat_lon = 1.0e-9;  // ~10 m - more trusted when SNS unavailable
            }
            double dop_factor = 1.0 / sqrt((double)valid_targets);
            measurement->noise[0] = measurement->noise[1] = base_noise_lat_lon * dop_factor;
            measurement->noise[2] = 400.0;  // ~20 m vertical (simplified, not using TCAS for altitude correction)

            // FIXED: Re-enable TCAS measurements with proper multilateration
            // Now using target positions stored in TCAS data for accurate position estimation
            // Only use TCAS for horizontal position (lat/lon), not altitude
            measurement->validity[0] = measurement->validity[1] = DATA_VALID;  // Lat/Lon only
            measurement->validity[2] = DATA_INVALID;  // Altitude - not using TCAS for this
        }
    }
}

double calculate_tcas_dop(const tcas_data_t* tcas_data,
    const geo_coordinates_t* own_position) {
    if (tcas_data->num_targets < 3) {
        return 999.9;  /* ����� ������ �������������� ������ */
    }

    /* ��������� �������������� ������ �������� ��� TCAS */
    /* �� ������ ������������ ����� ������������ ������������ ����� */

    double sum_sin_bearing = 0.0;
    double sum_cos_bearing = 0.0;

    for (uint32_t i = 0; i < tcas_data->num_targets; i++) {
        if (tcas_data->targets[i].validity == DATA_VALID) {
            double bearing_rad = tcas_data->targets[i].bearing * DEG_TO_RAD;
            sum_sin_bearing += sin(bearing_rad);
            sum_cos_bearing += cos(bearing_rad);
        }
    }

    /* �������������� ������ ������� �� ������������� �������� */
    double bearing_variance = (sum_sin_bearing * sum_sin_bearing + sum_cos_bearing * sum_cos_bearing) /
        (tcas_data->num_targets * tcas_data->num_targets);

    return 1.0 / sqrt(bearing_variance + 0.01);  /* ��������� ����� �������� ��� ��������� ������� �� ���� */
}

uint32_t generate_traffic_scenario(traffic_aircraft_t* traffic,
    uint32_t max_traffic,
    const approach_scenario_t* scenario,
    uint32_t num_traffic) {
    if (num_traffic > max_traffic) {
        num_traffic = max_traffic;
    }

    srand(time(NULL));

    for (uint32_t i = 0; i < num_traffic; i++) {
        traffic[i].id = 1000 + i;

        /* ���������� ��������� ������� ������ ��������� */
        double angle = (double)i / num_traffic * 2.0 * PI;
        double distance = 5000.0 + 15000.0 * (rand() / (double)RAND_MAX);  /* 5-20 �� */

        traffic[i].position.latitude = scenario->runway_threshold.latitude +
            (distance / EARTH_RADIUS) * RAD_TO_DEG * cos(angle);
        traffic[i].position.longitude = scenario->runway_threshold.longitude +
            (distance / EARTH_RADIUS) * RAD_TO_DEG * sin(angle) /
            cos(scenario->runway_threshold.latitude * DEG_TO_RAD);
        traffic[i].position.altitude = 1000.0 + 2000.0 * (rand() / (double)RAND_MAX);  /* 1-3 �� */

        /* ��������� �������� */
        traffic[i].velocity.vn = gaussian_random(0.0, 20.0);
        traffic[i].velocity.ve = gaussian_random(0.0, 20.0);
        traffic[i].velocity.vd = gaussian_random(0.0, 2.0);

        traffic[i].altitude_agl = traffic[i].position.altitude - 100.0;  /* ������������ ������ ����� 100 � */

        /* ��������� �������� ����� ADS-B � TCAS */
        traffic[i].has_adsb = (rand() % 100 < 80) ? DATA_VALID : DATA_INVALID;  /* 80% ����� ADS-B */
        traffic[i].has_tcas = DATA_VALID;  /* ��� ����� TCAS */
    }

    return num_traffic;
}

void calculate_approach_parameters(const geo_coordinates_t* position,
    const approach_scenario_t* scenario,
    double* distance_to_threshold,
    double* deviation_lateral,
    double* deviation_vertical) {
    /* ���������� �� ������ ��� */
    *distance_to_threshold = calculate_distance_2d(position, &scenario->runway_threshold);

    /* ������� ���������� �� ����� ��� */
    double bearing_to_threshold = calculate_bearing(position, &scenario->runway_threshold);
    double course_deviation = bearing_to_threshold - scenario->runway_heading;
    if (course_deviation > 180.0) course_deviation -= 360.0;
    if (course_deviation < -180.0) course_deviation += 360.0;

    *deviation_lateral = *distance_to_threshold * sin(course_deviation * DEG_TO_RAD);

    /* ������������ ���������� �� �������� */
    double expected_altitude = scenario->runway_threshold.altitude +
        *distance_to_threshold * tan(scenario->glide_slope * DEG_TO_RAD);
    *deviation_vertical = position->altitude - expected_altitude;
}

uint32_t simulate_approach(const simulation_config_t* config, approach_scenario_t scenario,
    simulation_result_t* results,
    uint32_t max_results) {
    uint32_t num_results = 0;
    double time = 0.0;

    /* ������������� ������� */
    traffic_aircraft_t traffic[20];
    uint32_t num_traffic = generate_traffic_scenario(traffic, 20, &config->scenario, config->num_traffic);

    /* ��������� ������� - �� 20 �� �� ������ ��� */
    geo_coordinates_t true_position;
    true_position.latitude = config->scenario.runway_threshold.latitude -
        (20000.0 / EARTH_RADIUS) * RAD_TO_DEG * cos(config->scenario.runway_heading * DEG_TO_RAD);
    true_position.longitude = config->scenario.runway_threshold.longitude -
        (20000.0 / EARTH_RADIUS) * RAD_TO_DEG * sin(config->scenario.runway_heading * DEG_TO_RAD);
    true_position.altitude = config->scenario.runway_threshold.altitude +
        20000.0 * tan(config->scenario.glide_slope * DEG_TO_RAD);

    velocity_t true_velocity;
    true_velocity.vn = scenario.approach_speed * cos(config->scenario.runway_heading * DEG_TO_RAD);
    true_velocity.ve = scenario.approach_speed * sin(config->scenario.runway_heading * DEG_TO_RAD);
    true_velocity.vd = -scenario.approach_speed * tan(config->scenario.glide_slope * DEG_TO_RAD);

    orientation_t orientation;
    orientation.heading = config->scenario.runway_heading;
    orientation.pitch = -config->scenario.glide_slope;
    orientation.roll = 0.0;

    /* ������������� ������� ������� */
    kalman_state_t kalman_state;
    kalman_init(&kalman_state, &true_position, &true_velocity, &orientation);

    /* �������� ���� ������������� */
    while (time < config->simulation_time && num_results < max_results) {
        /* ��������� ����������� ��� */
        /* Для режима "Без TCAS": после 300 секунды СНС полностью недоступна */
        /* Для режима "С TCAS": после 300 секунды СНС доступна, но с помехами (увеличенные шумы) */
    data_validity_t sns_available = DATA_VALID;
    int sns_has_interference = 0;  /* Флаг помех СНС (для режима с TCAS) */
    if (config->sns_outage_duration > 0.0 &&
        time >= config->sns_outage_start &&
        time < config->sns_outage_start + config->sns_outage_duration) {
            if (config->use_tcas == DATA_VALID) {
                /* Режим с TCAS: СНС доступна, но с помехами */
                sns_available = DATA_VALID;
                sns_has_interference = 1;
            } else {
                /* Режим без TCAS: СНС полностью недоступна */
                sns_available = DATA_INVALID;
            }
        }

        /* ���������� ������ �� �������� */
        ins_data_t ins_data;
        sns_data_t sns_data;
        tcas_data_t tcas_data;
        koei_data_t koei_data;
        geo_coordinates_t estimated_position;
        
        generate_ins_data(&ins_data, &true_position, &true_velocity,
            config->ins_drift_rate, time);
        generate_sns_data(&sns_data, &true_position, &true_velocity,
            1.5, 2.0, sns_available, time, sns_has_interference);
        
        /* ������������ ��������� ������� (������� ����������� �������) */
        estimated_position.latitude = kalman_state.state[0];
        estimated_position.longitude = kalman_state.state[1];
        estimated_position.altitude = kalman_state.state[2];
        
        /* ВАЖНО: TCAS должен измерять от истинного положения самолета.
           Это правильно для мультилатерации - мы сравниваем измеренные дальности
           (от истинного положения) с ожидаемыми (от оценки), чтобы получить
           поправку к оценке положения. */
        if (config->use_tcas == DATA_VALID) {
            generate_tcas_data(&tcas_data, &true_position, traffic, num_traffic, time);
        }
        else {
            tcas_data.validity = DATA_INVALID;
            tcas_data.num_targets = 0;
        }
        
        if (config->use_koei == DATA_VALID) {
            generate_koei_data(&koei_data, &true_position, 100.0, time);
        }
        else {
            koei_data.validity = DATA_INVALID;
        }
        
        /* ��������� ������ ��������� */
        measurement_vector_t measurement;
        form_measurement_vector(&measurement, &ins_data, &sns_data, &tcas_data, &koei_data,
            &estimated_position);

        /* ������������ � ���������� ������� */
        kalman_predict(&kalman_state, config->time_step);
        
        // FIXED: Add INS drift directly to state when no external corrections available
        // This models drift accumulation in INS-only operation
        int has_corrections = (sns_available == DATA_VALID) || 
                             (koei_data.validity == DATA_VALID && koei_data.correlation_coefficient > 0.6) ||
                             (tcas_data.validity == DATA_VALID && tcas_data.num_targets >= 3);
        
        static int drift_initialized = 0;
        static double drift_direction_n = 0.0;
        static double drift_direction_e = 0.0;
        static double velocity_bias_n = 0.0;
        static double velocity_bias_e = 0.0;
        static double altitude_bias_rate = 0.0;

        if (has_corrections) {
            drift_initialized = 0;
        }

        if (!has_corrections) {
            // FIXED: Improved INS drift model - bias-driven drift accumulates over time
            // Interpret ins_drift_rate as meters of drift per hour caused by INS biases
            if (!drift_initialized) {
                double bias_std_mps = config->ins_drift_rate / 3600.0;  // Convert to m/s bias magnitude
                drift_direction_n = gaussian_random(0.0, 1.0);
                drift_direction_e = gaussian_random(0.0, 1.0);
                double norm = sqrt(drift_direction_n * drift_direction_n + drift_direction_e * drift_direction_e);
                if (norm < 1e-6) norm = 1.0;
                drift_direction_n /= norm;
                drift_direction_e /= norm;

                velocity_bias_n = bias_std_mps * drift_direction_n;
                velocity_bias_e = bias_std_mps * drift_direction_e;
                altitude_bias_rate = bias_std_mps * 0.1 * gaussian_random(0.0, 1.0);
                drift_initialized = 1;
            }

            // Apply velocity bias (integrates to position drift)
            kalman_state.state[3] += velocity_bias_n * config->time_step;
            kalman_state.state[4] += velocity_bias_e * config->time_step;

            // Integrate velocity bias to position (quadratic growth)
            double lat_bias_deg = (velocity_bias_n * config->time_step / EARTH_RADIUS) * RAD_TO_DEG;
            double lon_bias_deg = (velocity_bias_e * config->time_step /
                                   (EARTH_RADIUS * cos(kalman_state.state[0] * DEG_TO_RAD))) * RAD_TO_DEG;
            kalman_state.state[0] += lat_bias_deg;
            kalman_state.state[1] += lon_bias_deg;

            // Altitude drift
            kalman_state.state[2] += altitude_bias_rate * config->time_step;
        }
        
        kalman_update(&kalman_state, &measurement, config->use_tcas == DATA_VALID);

        /* ��������� ���������� */
        results[num_results].timestamp = time;
        results[num_results].estimated_position.latitude = kalman_state.state[0];
        results[num_results].estimated_position.longitude = kalman_state.state[1];
        results[num_results].estimated_position.altitude = kalman_state.state[2];
        results[num_results].true_position = true_position;

        /* ��������� ������ */
        results[num_results].position_error_2d = calculate_distance_2d(
            &results[num_results].estimated_position, &true_position);
        results[num_results].position_error_3d = calculate_distance_3d(
            &results[num_results].estimated_position, &true_position);

        velocity_t estimated_velocity;
        estimated_velocity.vn = kalman_state.state[3];
        estimated_velocity.ve = kalman_state.state[4];
        estimated_velocity.vd = kalman_state.state[5];

        double dv2 = (estimated_velocity.vn - true_velocity.vn) * (estimated_velocity.vn - true_velocity.vn) +
            (estimated_velocity.ve - true_velocity.ve) * (estimated_velocity.ve - true_velocity.ve) +
            (estimated_velocity.vd - true_velocity.vd) * (estimated_velocity.vd - true_velocity.vd);
        results[num_results].velocity_error = sqrt(dv2);

        /* �������������� ������� */
        results[num_results].hdop = sns_data.hdop;
        results[num_results].vdop = sns_data.vdop;
        results[num_results].num_tcas_targets = tcas_data.num_targets;
        results[num_results].sns_available = sns_available;
        results[num_results].tcas_available = (tcas_data.num_targets > 0) ? DATA_VALID : DATA_INVALID;

        /* ��������� ������� ��� ���������� ���� */
        double dt = config->time_step;
        true_position.latitude += (true_velocity.vn / EARTH_RADIUS) * RAD_TO_DEG * dt;
        true_position.longitude += (true_velocity.ve / EARTH_RADIUS) * RAD_TO_DEG * dt /
            cos(true_position.latitude * DEG_TO_RAD);
        true_position.altitude += true_velocity.vd * dt;

        time += dt;
        num_results++;
    }

    return num_results;
}

int save_simulation_results(const simulation_result_t* results,
    uint32_t num_results,
    const char* filename) {
    FILE* file = fopen(filename, "w");
    if (!file) {
        return -1;
    }

    /* ���������� ��������� */
    fprintf(file, "timestamp,est_lat,est_lon,est_alt,true_lat,true_lon,true_alt,"
        "error_2d,error_3d,velocity_error,hdop,vdop,tcas_targets,sns_available,tcas_available\n");

    /* ���������� ������ */
    for (uint32_t i = 0; i < num_results; i++) {
        fprintf(file, "%.2f,%.8f,%.8f,%.1f,%.8f,%.8f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%d,%d\n",
            results[i].timestamp,
            results[i].estimated_position.latitude,
            results[i].estimated_position.longitude,
            results[i].estimated_position.altitude,
            results[i].true_position.latitude,
            results[i].true_position.longitude,
            results[i].true_position.altitude,
            results[i].position_error_2d,
            results[i].position_error_3d,
            results[i].velocity_error,
            results[i].hdop,
            results[i].vdop,
            results[i].num_tcas_targets,
            results[i].sns_available,
            results[i].tcas_available);
    }

    fclose(file);
    return 0;
}

void print_simulation_statistics(const simulation_result_t* results,
    uint32_t num_results) {
    if (num_results == 0) {
        printf("��� ������ ��� �������\n");
        return;
    }

    /* ��������� ���������� */
    double sum_error_2d = 0.0, sum_error_3d = 0.0, sum_velocity_error = 0.0;
    double max_error_2d = 0.0, max_error_3d = 0.0;
    uint32_t count_sns_loss = 0, count_tcas_available = 0;

    for (uint32_t i = 0; i < num_results; i++) {
        sum_error_2d += results[i].position_error_2d;
        sum_error_3d += results[i].position_error_3d;
        sum_velocity_error += results[i].velocity_error;

        if (results[i].position_error_2d > max_error_2d) {
            max_error_2d = results[i].position_error_2d;
        }
        if (results[i].position_error_3d > max_error_3d) {
            max_error_3d = results[i].position_error_3d;
        }

        if (results[i].sns_available == DATA_INVALID) {
            count_sns_loss++;
        }
        if (results[i].tcas_available == DATA_VALID) {
            count_tcas_available++;
        }
    }

    double avg_error_2d = sum_error_2d / num_results;
    double avg_error_3d = sum_error_3d / num_results;
    double avg_velocity_error = sum_velocity_error / num_results;

    /* ������� ���������� */
    printf("\n=== ���������� ������������� ===\n");
    printf("���������� ���������: %u\n", num_results);
    printf("������� 2D ������ ����������������: %.2f �\n", avg_error_2d);
    printf("������� 3D ������ ����������������: %.2f �\n", avg_error_3d);
    printf("������������ 2D ������: %.2f �\n", max_error_2d);
    printf("������������ 3D ������: %.2f �\n", max_error_3d);
    printf("������� ������ ��������: %.2f �/�\n", avg_velocity_error);
    printf("����� ������ ������� ���: %.1f%%\n",
        (count_sns_loss * 100.0) / num_results);
    printf("����������� TCAS: %.1f%%\n",
        (count_tcas_available * 100.0) / num_results);
}

void compare_tcas_integration(const simulation_result_t* results_with_tcas,
    const simulation_result_t* results_without_tcas,
    uint32_t num_results) {
    printf("\n=== ��������� ����������� ���������� TCAS ===\n");

    /* ��������� ������� ������ ��� ����� ��������� */
    double error_2d_with = 0.0, error_2d_without = 0.0;
    double error_3d_with = 0.0, error_3d_without = 0.0;

    for (uint32_t i = 0; i < num_results; i++) {
        error_2d_with += results_with_tcas[i].position_error_2d;
        error_2d_without += results_without_tcas[i].position_error_2d;
        error_3d_with += results_with_tcas[i].position_error_3d;
        error_3d_without += results_without_tcas[i].position_error_3d;
    }

    error_2d_with /= num_results;
    error_2d_without /= num_results;
    error_3d_with /= num_results;
    error_3d_without /= num_results;

    printf("������� 2D ������ ��� TCAS: %.2f �\n", error_2d_without);
    printf("������� 2D ������ � TCAS: %.2f �\n", error_2d_with);
    printf("��������� 2D ��������: %.2f%%\n",
        ((error_2d_without - error_2d_with) / error_2d_without) * 100.0);

    printf("������� 3D ������ ��� TCAS: %.2f �\n", error_3d_without);
    printf("������� 3D ������ � TCAS: %.2f �\n", error_3d_with);
    printf("��������� 3D ��������: %.2f%%\n",
        ((error_3d_without - error_3d_with) / error_3d_without) * 100.0);
}