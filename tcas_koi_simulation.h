/**
 * \file tcas_koi_simulation.h
 * \brief Функции моделирования интеграции TCAS в систему КОИ
 * \details Заголовочный файл с прототипами функций для моделирования
 *          комплексной обработки навигационной информации
 */

#ifndef TCAS_KOI_SIMULATION_H
#define TCAS_KOI_SIMULATION_H

#include "tcas_koi_types.h"

 /**
  * \brief Инициализация состояния фильтра Калмана
  * \param state Указатель на структуру состояния фильтра
  * \param initial_position Начальная позиция
  * \param initial_velocity Начальная скорость
  * \param initial_orientation Начальная ориентация
  */
void kalman_init(kalman_state_t* state,
    const geo_coordinates_t* initial_position,
    const velocity_t* initial_velocity,
    const orientation_t* initial_orientation);

/**
 * \brief Предсказание состояния фильтра Калмана
 * \param state Указатель на структуру состояния фильтра
 * \param dt Временной интервал предсказания, с
 */
void kalman_predict(kalman_state_t* state, double dt);

/**
 * \brief Обновление состояния фильтра Калмана измерениями
 * \param state Указатель на структуру состояния фильтра
 * \param measurement Вектор измерений
 */
void kalman_update(kalman_state_t* state, const measurement_vector_t* measurement);

/**
 * \brief Генерация данных ИНС с учетом дрейфа
 * \param ins_data Указатель на структуру данных ИНС
 * \param true_position Истинная позиция
 * \param true_velocity Истинная скорость
 * \param drift_rate Скорость дрейфа, м/с/час
 * \param time Время моделирования, с
 */
void generate_ins_data(ins_data_t* ins_data,
    const geo_coordinates_t* true_position,
    const velocity_t* true_velocity,
    double drift_rate,
    double time);

/**
 * \brief Генерация данных СНС с учетом помех
 * \param sns_data Указатель на структуру данных СНС
 * \param true_position Истинная позиция
 * \param true_velocity Истинная скорость
 * \param hdop Горизонтальный геометрический фактор
 * \param vdop Вертикальный геометрический фактор
 * \param sns_available Доступность сигнала СНС
 * \param time Время моделирования, с
 */
void generate_sns_data(sns_data_t* sns_data,
    const geo_coordinates_t* true_position,
    const velocity_t* true_velocity,
    double hdop, double vdop,
    data_validity_t sns_available,
    double time);

/**
 * \brief Генерация данных TCAS для окружающих воздушных судов
 * \param tcas_data Указатель на структуру данных TCAS
 * \param own_position Позиция собственного воздушного судна
 * \param traffic Массив данных других воздушных судов
 * \param num_traffic Количество других воздушных судов
 * \param time Время моделирования, с
 */
void generate_tcas_data(tcas_data_t* tcas_data,
    const geo_coordinates_t* own_position,
    const traffic_aircraft_t* traffic,
    uint32_t num_traffic,
    double time);

/**
 * \brief Генерация данных КЭОИ
 * \param koei_data Указатель на структуру данных КЭОИ
 * \param true_position Истинная позиция
 * \param terrain_height Высота рельефа местности
 * \param time Время моделирования, с
 */
void generate_koei_data(koei_data_t* koei_data,
    const geo_coordinates_t* true_position,
    double terrain_height,
    double time);

/**
 * \brief Формирование вектора измерений для фильтра Калмана
 * \param measurement Указатель на вектор измерений
 * \param ins_data Данные ИНС
 * \param sns_data Данные СНС
 * \param tcas_data Данные TCAS
 * \param koei_data Данные КЭОИ
 * \param estimated_position Оцененная позиция
 */
void form_measurement_vector(measurement_vector_t* measurement,
    const ins_data_t* ins_data,
    const sns_data_t* sns_data,
    const tcas_data_t* tcas_data,
    const koei_data_t* koei_data,
    const geo_coordinates_t* estimated_position);

/**
 * \brief Вычисление геометрического фактора точности для TCAS
 * \param tcas_data Данные TCAS
 * \param own_position Позиция собственного воздушного судна
 * \return Геометрический фактор точности TCAS
 */
double calculate_tcas_dop(const tcas_data_t* tcas_data,
    const geo_coordinates_t* own_position);

/**
 * \brief Симуляция захода на посадку
 * \param config Конфигурация моделирования
 * \param results Указатель на массив результатов
 * \param max_results Максимальное количество результатов
 * \return Количество сгенерированных результатов
 */
uint32_t simulate_approach(const simulation_config_t* config, approach_scenario_t scenario,
    simulation_result_t* results,
    uint32_t max_results);

/**
 * \brief Генерация трафика для сцены захода на посадку
 * \param traffic Указатель на массив данных трафика
 * \param max_traffic Максимальное количество воздушных судов
 * \param scenario Сценарий захода на посадку
 * \param num_traffic Необходимое количество воздушных судов
 * \return Количество сгенерированных воздушных судов
 */
uint32_t generate_traffic_scenario(traffic_aircraft_t* traffic,
    uint32_t max_traffic,
    const approach_scenario_t* scenario,
    uint32_t num_traffic);

/**
 * \brief Вычисление параметров захода на посадку
 * \param position Текущая позиция
 * \param scenario Сценарий захода на посадку
 * \param distance_to_threshold Расстояние до порога ВПП, м
 * \param deviation_lateral Боковое отклонение от курса, м
 * \param deviation_vertical Вертикальное отклонение от глиссады, м
 */
void calculate_approach_parameters(const geo_coordinates_t* position,
    const approach_scenario_t* scenario,
    double* distance_to_threshold,
    double* deviation_lateral,
    double* deviation_vertical);

/**
 * \brief Сохранение результатов моделирования в файл
 * \param results Массив результатов
 * \param num_results Количество результатов
 * \param filename Имя файла для сохранения
 * \return 0 в случае успеха, -1 в случае ошибки
 */
int save_simulation_results(const simulation_result_t* results,
    uint32_t num_results,
    const char* filename);

/**
 * \brief Загрузка конфигурации моделирования из файла
 * \param config Указатель на структуру конфигурации
 * \param filename Имя файла конфигурации
 * \return 0 в случае успеха, -1 в случае ошибки
 */
int load_simulation_config(simulation_config_t* config,
    const char* filename);

/**
 * \brief Вывод статистики моделирования
 * \param results Массив результатов
 * \param num_results Количество результатов
 */
void print_simulation_statistics(const simulation_result_t* results,
    uint32_t num_results);

/**
 * \brief Сравнение результатов моделирования с и без TCAS
 * \param results_with_tcas Результаты с TCAS
 * \param results_without_tcas Результаты без TCAS
 * \param num_results Количество результатов
 */
void compare_tcas_integration(const simulation_result_t* results_with_tcas,
    const simulation_result_t* results_without_tcas,
    uint32_t num_results);

#endif /* TCAS_KOI_SIMULATION_H */