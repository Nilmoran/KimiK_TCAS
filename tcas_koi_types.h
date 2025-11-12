#pragma once
/**
 * \file tcas_koi_types.h
 * \brief Типы данных для моделирования интеграции TCAS в систему КОИ
 * \details Определяет основные типы данных для моделирования интеграции навигационных систем
 *          и использования данных TCAS для подхода на посадку
 */

#ifndef TCAS_KOI_TYPES_H
#define TCAS_KOI_TYPES_H

#include <stdint.h>
#include <math.h>

 /* Константы */
#define PI 3.14159265358979323846
#define DEG_TO_RAD (PI / 180.0)
#define RAD_TO_DEG (180.0 / PI)
#define EARTH_RADIUS 6378137.0 /* Радиус Земли в метрах */
#define LIGHT_SPEED 299792458.0 /* Скорость света в м/с */

/* Тип валидности данных */
typedef enum {
    DATA_INVALID = 0,
    DATA_VALID = 1
} data_validity_t;

/* Тип источника навигационных данных */
typedef enum {
    SOURCE_INS = 0,    /* ИНС */
    SOURCE_SNS = 1,    /* СНС */
    SOURCE_TCAS = 2,   /* TCAS */
    SOURCE_KOEI = 3,   /* КЭОИ */
    SOURCE_BARO = 4,   /* Барометрический */
    SOURCE_RADAR = 5   /* Радиолокационный */
} nav_source_t;

/* Географические координаты в градусах */
typedef struct {
    double latitude;   /* Широта, градусы */
    double longitude;  /* Долгота, градусы */
    double altitude;   /* Высота, метры */
} geo_coordinates_t;

/* Географические координаты в прямоугольной системе */
typedef struct {
    double x;  /* Восточная координата, м */
    double y;  /* Северная координата, м */
    double z;  /* Вертикальная координата, м */
} cartesian_coordinates_t;

/* Вектор скорости */
typedef struct {
    double vn;  /* Северная составляющая скорости, м/с */
    double ve;  /* Восточная составляющая скорости, м/с */
    double vd;  /* Вертикальная составляющая скорости, м/с */
} velocity_t;

/* Углы ориентации */
typedef struct {
    double heading;    /* Курс, градусы */
    double pitch;      /* Тангаж, градусы */
    double roll;       /* Крен, градусы */
} orientation_t;

/* Данные ИНС */
typedef struct {
    geo_coordinates_t position;
    velocity_t velocity;
    orientation_t orientation;
    data_validity_t validity;
    double timestamp;
} ins_data_t;

/* Данные СНС */
typedef struct {
    geo_coordinates_t position;
    velocity_t velocity;
    data_validity_t validity;
    double hdop;  /* Горизонтальный геометрический фактор */
    double vdop;  /* Вертикальный геометрический фактор */
    double timestamp;
} sns_data_t;

/* Данные TCAS для одного воздушного судна */
typedef struct {
    uint32_t aircraft_id;    /* Идентификатор воздушного судна */
    double range;            /* Расстояние до цели, м */
    double bearing;          /* Пеленг на цель, градусы */
    double relative_altitude; /* Относительная высота, м */
    double closure_rate;     /* Скорость сближения, м/с */
    geo_coordinates_t target_position;  /* FIXED: Store target position for multilateration */
    data_validity_t validity;
    double timestamp;
} tcas_target_t;

/* Данные TCAS */
typedef struct {
    tcas_target_t targets[10]; /* Массив обнаруженных целей */
    uint32_t num_targets;      /* Количество обнаруженных целей */
    data_validity_t validity;
    double timestamp;
} tcas_data_t;

/* Данные КЭОИ (корреляционно-экстремальная обработка изображений) */
typedef struct {
    geo_coordinates_t position;
    data_validity_t validity;
    double correlation_coefficient;  /* Коэффициент корреляции */
    double timestamp;
} koei_data_t;

/* Данные барометрического высотомера */
typedef struct {
    double altitude;  /* Барометрическая высота, м */
    data_validity_t validity;
    double timestamp;
} baro_data_t;

/* Данные радиолокационного высотомера */
typedef struct {
    double altitude;  /* Радиолокационная высота, м */
    data_validity_t validity;
    double timestamp;
} radar_data_t;

/* Вектор измерений для фильтра Калмана */
typedef struct {
    nav_source_t source;
    double measurement[6];  /* [delta_lat, delta_lon, delta_alt, delta_vn, delta_ve, delta_vd] */
    double noise[6];        /* Дисперсия шума измерений */
    data_validity_t validity[6];
} measurement_vector_t;

/* Состояние фильтра Калмана */
typedef struct {
    double state[15];  /* [lat, lon, alt, vn, ve, vd, roll, pitch, heading,
                         bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz] */
    double covariance[15][15];  /* Матрица ковариации */
    double timestamp;
} kalman_state_t;

/* Параметры сценария подхода на посадку */
typedef struct {
    char name[100];             /* Название сценария */
    geo_coordinates_t runway_threshold;  /* Координаты порога ВПП */
    double runway_heading;      /* Курс ВПП, градусы */
    double glide_slope;         /* Угол глиссады, градусы */
    double decision_height;     /* Высота принятия решения, м */
    double approach_speed;      /* Скорость подхода на посадку, м/с */
} approach_scenario_t;

/* Параметры воздушного судна в трафике */
typedef struct {
    uint32_t id;
    geo_coordinates_t position;
    velocity_t velocity;
    double altitude_agl;  /* Высота над уровнем земли, м */
    data_validity_t has_adsb;  /* Наличие ADS-B */
    data_validity_t has_tcas;  /* Наличие TCAS */
} traffic_aircraft_t;

/* Результаты моделирования */
typedef struct {
    double timestamp;
    geo_coordinates_t estimated_position;
    geo_coordinates_t true_position;
    double position_error_2d;  /* 2D ошибка позиционирования, м */
    double position_error_3d;  /* 3D ошибка позиционирования, м */
    double velocity_error;     /* Ошибка скорости, м/с */
    double hdop;  /* Горизонтальный геометрический фактор */
    double vdop;  /* Вертикальный геометрический фактор */
    uint32_t num_tcas_targets;  /* Количество целей TCAS */
    data_validity_t sns_available;
    data_validity_t tcas_available;
} simulation_result_t;

/* Конфигурация моделирования */
typedef struct {
    double simulation_time;     /* Время моделирования, с */
    double time_step;          /* Шаг моделирования, с */
    approach_scenario_t scenario;  /* Сценарий подхода на посадку */
    uint32_t num_traffic;       /* Количество целей в трафике */
    double ins_drift_rate;      /* Скорость дрейфа ИНС, м/ч/ч */
    double sns_outage_start;    /* Время начала потери сигнала СНС, с */
    double sns_outage_duration;  /* Продолжительность потери сигнала СНС, с */
    data_validity_t use_tcas;   /* Использование TCAS в моделировании */
    data_validity_t use_koei;   /* Использование КЭОИ в моделировании */
} simulation_config_t;

/* Функции для работы с координатами */
static inline void geo_to_cartesian(const geo_coordinates_t* geo,
    cartesian_coordinates_t* cart) {
    double cos_lat = cos(geo->latitude * DEG_TO_RAD);
    double sin_lat = sin(geo->latitude * DEG_TO_RAD);
    double cos_lon = cos(geo->longitude * DEG_TO_RAD);
    double sin_lon = sin(geo->longitude * DEG_TO_RAD);

    cart->x = EARTH_RADIUS * cos_lat * cos_lon;
    cart->y = EARTH_RADIUS * cos_lat * sin_lon;
    cart->z = EARTH_RADIUS * sin_lat;
}

static inline double calculate_distance_2d(const geo_coordinates_t* pos1,
    const geo_coordinates_t* pos2) {
    double dlat = (pos2->latitude - pos1->latitude) * DEG_TO_RAD;
    double dlon = (pos2->longitude - pos1->longitude) * DEG_TO_RAD;
    double lat1 = pos1->latitude * DEG_TO_RAD;
    double lat2 = pos2->latitude * DEG_TO_RAD;

    double a = sin(dlat / 2) * sin(dlat / 2) +
        cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return EARTH_RADIUS * c;
}

static inline double calculate_distance_3d(const geo_coordinates_t* pos1,
    const geo_coordinates_t* pos2) {
    double distance_2d = calculate_distance_2d(pos1, pos2);
    double altitude_diff = pos2->altitude - pos1->altitude;

    return sqrt(distance_2d * distance_2d + altitude_diff * altitude_diff);
}

static inline double calculate_bearing(const geo_coordinates_t* from,
    const geo_coordinates_t* to) {
    double lat1 = from->latitude * DEG_TO_RAD;
    double lat2 = to->latitude * DEG_TO_RAD;
    double dlon = (to->longitude - from->longitude) * DEG_TO_RAD;

    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    double bearing = atan2(y, x) * RAD_TO_DEG;

    return fmod(bearing + 360.0, 360.0);
}

#endif /* TCAS_KOI_TYPES_H */
