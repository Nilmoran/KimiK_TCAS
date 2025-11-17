/**
 * \file tcas_koi_simulation.h
 * \brief ������� ������������� ���������� TCAS � ������� ���
 * \details ������������ ���� � ����������� ������� ��� �������������
 *          ����������� ��������� ������������� ����������
 */

#ifndef TCAS_KOI_SIMULATION_H
#define TCAS_KOI_SIMULATION_H

#include "tcas_koi_types.h"

 /**
  * \brief ������������� ��������� ������� �������
  * \param state ��������� �� ��������� ��������� �������
  * \param initial_position ��������� �������
  * \param initial_velocity ��������� ��������
  * \param initial_orientation ��������� ����������
  */
void kalman_init(kalman_state_t* state,
    const geo_coordinates_t* initial_position,
    const velocity_t* initial_velocity,
    const orientation_t* initial_orientation);

/**
 * \brief ������������ ��������� ������� �������
 * \param state ��������� �� ��������� ��������� �������
 * \param dt ��������� �������� ������������, �
 */
void kalman_predict(kalman_state_t* state, double dt);

/**
 * \brief ���������� ��������� ������� ������� �����������
 * \param state ��������� �� ��������� ��������� �������
 * \param measurement ������ ���������
 */
void kalman_update(kalman_state_t* state, const measurement_vector_t* measurement, int use_tcas);

/**
 * \brief ��������� ������ ��� � ������ ������
 * \param ins_data ��������� �� ��������� ������ ���
 * \param true_position �������� �������
 * \param true_velocity �������� ��������
 * \param drift_rate �������� ������, �/�/���
 * \param time ����� �������������, �
 */
void generate_ins_data(ins_data_t* ins_data,
    const geo_coordinates_t* true_position,
    const velocity_t* true_velocity,
    double drift_rate,
    double time);

/**
 * \brief ��������� ������ ��� � ������ �����
 * \param sns_data ��������� �� ��������� ������ ���
 * \param true_position �������� �������
 * \param true_velocity �������� ��������
 * \param hdop �������������� �������������� ������
 * \param vdop ������������ �������������� ������
 * \param sns_available ����������� ������� ���
 * \param time ����� �������������, �
 */
void generate_sns_data(sns_data_t* sns_data,
    const geo_coordinates_t* true_position,
    const velocity_t* true_velocity,
    double hdop, double vdop,
    data_validity_t sns_available,
    double time,
    int has_interference);

/**
 * \brief ��������� ������ TCAS ��� ���������� ��������� �����
 * \param tcas_data ��������� �� ��������� ������ TCAS
 * \param own_position ������� ������������ ���������� �����
 * \param traffic ������ ������ ������ ��������� �����
 * \param num_traffic ���������� ������ ��������� �����
 * \param time ����� �������������, �
 */
void generate_tcas_data(tcas_data_t* tcas_data,
    const geo_coordinates_t* own_position,
    const traffic_aircraft_t* traffic,
    uint32_t num_traffic,
    double time);

/**
 * \brief ��������� ������ ����
 * \param koei_data ��������� �� ��������� ������ ����
 * \param true_position �������� �������
 * \param terrain_height ������ ������� ���������
 * \param time ����� �������������, �
 */
void generate_koei_data(koei_data_t* koei_data,
    const geo_coordinates_t* true_position,
    double terrain_height,
    double time);

/**
 * \brief ������������ ������� ��������� ��� ������� �������
 * \param measurement ��������� �� ������ ���������
 * \param ins_data ������ ���
 * \param sns_data ������ ���
 * \param tcas_data ������ TCAS
 * \param koei_data ������ ����
 * \param estimated_position ��������� �������
 */
void form_measurement_vector(measurement_vector_t* measurement,
    const ins_data_t* ins_data,
    const sns_data_t* sns_data,
    const tcas_data_t* tcas_data,
    const koei_data_t* koei_data,
    const geo_coordinates_t* estimated_position);

/**
 * \brief ���������� ��������������� ������� �������� ��� TCAS
 * \param tcas_data ������ TCAS
 * \param own_position ������� ������������ ���������� �����
 * \return �������������� ������ �������� TCAS
 */
double calculate_tcas_dop(const tcas_data_t* tcas_data,
    const geo_coordinates_t* own_position);

/**
 * \brief ��������� ������ �� �������
 * \param config ������������ �������������
 * \param results ��������� �� ������ �����������
 * \param max_results ������������ ���������� �����������
 * \return ���������� ��������������� �����������
 */
uint32_t simulate_approach(const simulation_config_t* config, approach_scenario_t scenario,
    simulation_result_t* results,
    uint32_t max_results);

/**
 * \brief ��������� ������� ��� ����� ������ �� �������
 * \param traffic ��������� �� ������ ������ �������
 * \param max_traffic ������������ ���������� ��������� �����
 * \param scenario �������� ������ �� �������
 * \param num_traffic ����������� ���������� ��������� �����
 * \return ���������� ��������������� ��������� �����
 */
uint32_t generate_traffic_scenario(traffic_aircraft_t* traffic,
    uint32_t max_traffic,
    const approach_scenario_t* scenario,
    uint32_t num_traffic);

/**
 * \brief ���������� ���������� ������ �� �������
 * \param position ������� �������
 * \param scenario �������� ������ �� �������
 * \param distance_to_threshold ���������� �� ������ ���, �
 * \param deviation_lateral ������� ���������� �� �����, �
 * \param deviation_vertical ������������ ���������� �� ��������, �
 */
void calculate_approach_parameters(const geo_coordinates_t* position,
    const approach_scenario_t* scenario,
    double* distance_to_threshold,
    double* deviation_lateral,
    double* deviation_vertical);

/**
 * \brief ���������� ����������� ������������� � ����
 * \param results ������ �����������
 * \param num_results ���������� �����������
 * \param filename ��� ����� ��� ����������
 * \return 0 � ������ ������, -1 � ������ ������
 */
int save_simulation_results(const simulation_result_t* results,
    uint32_t num_results,
    const char* filename);

/**
 * \brief �������� ������������ ������������� �� �����
 * \param config ��������� �� ��������� ������������
 * \param filename ��� ����� ������������
 * \return 0 � ������ ������, -1 � ������ ������
 */
int load_simulation_config(simulation_config_t* config,
    const char* filename);

/**
 * \brief ����� ���������� �������������
 * \param results ������ �����������
 * \param num_results ���������� �����������
 */
void print_simulation_statistics(const simulation_result_t* results,
    uint32_t num_results);

/**
 * \brief ��������� ����������� ������������� � � ��� TCAS
 * \param results_with_tcas ���������� � TCAS
 * \param results_without_tcas ���������� ��� TCAS
 * \param num_results ���������� �����������
 */
void compare_tcas_integration(const simulation_result_t* results_with_tcas,
    const simulation_result_t* results_without_tcas,
    uint32_t num_results);

#endif /* TCAS_KOI_SIMULATION_H */