
#include "ecatslave.h"
#include "ecatmaster.h"

ecatslave::ecatslave()
{


    memset(slave_domain_out_regs, 0 , sizeof(ec_pdo_entry_reg_t) * MAX_ENTRIES);
    memset(slave_domain_in_regs, 0 , sizeof(ec_pdo_entry_reg_t) * MAX_ENTRIES);


    memset(v_global_offset_buffer, 0 , sizeof(unsigned int) * MAX_ENTRIES);
    memset(v_global_entries, 0 , sizeof(ec_pdo_entry_info_t) * MAX_ENTRIES);
    memset(v_global_pdos, 0 , sizeof(ec_pdo_info_t) * MAX_PDOS);
    memset(v_global_slave_syncs, 0 , sizeof(ec_sync_info_t) * MAX_SYNC);



//    memset(v_global_vendor_name, 0 , MAX_NAME_LEN);
//    memset(v_global_slave_models, 0 , MAX_NAME_LEN);
    v_global_slave_models = "";
    v_global_vendor_name = "";

    v_global_vendor_id = 0;
    v_global_product_id = 0;
}




/**
  * 1. EtherCAT slave Initialize part
  *
*/



void ecatslave::f_init_initialize(std::string v_local_slave_models,
                                        std::string v_local_vendor_name,
                                        uint32_t v_local_vendorID,
                                        uint32_t v_local_productCODE,
                                        int v_local_slave_position,
                                        bool v_local_is_servo_drive)
{
    unsigned int v_area_count = 0;
    unsigned int v_area_pdos_count = 0;
    unsigned int v_area_pdos_limit = 0;
    unsigned int v_area_entries_count = 0;
    unsigned int v_area_entries_limit = 0;
    unsigned int v_area_total_entries_count = 0;


    unsigned int v_area_inreg_count = 0;
    unsigned int v_area_outreg_count = 0;



    // pointer
    ec_pdo_info_t *v_area_ptr_pdos_temp;
    ec_pdo_entry_info_t *v_area_ptr_etries_temp;



#if ETHERCAT_SLAVE_CLASS_DEBUG
    d_print("[CLASS]ethercatslave :: f_init_initialize start\r\n");
#endif

    v_global_is_servo_drive = v_local_is_servo_drive;
    v_global_vendor_id = v_local_vendorID;
    v_global_product_id = v_local_productCODE;

//    while(v_local_slave_models.c_str()[v_area_count] != '\0')v_area_count++;
//    memcpy(v_global_slave_models,v_local_slave_models,v_area_count);
    if(v_local_slave_models.length() > 0){
        v_global_slave_models = v_local_slave_models;
    }

    v_area_count = 0;

//    while(v_local_vendor_name.c_str()[v_area_count] != '\0')v_area_count++;
//    memcpy(v_global_vendor_name,v_local_vendor_name,v_area_count);
    if(v_local_vendor_name.length() > 0){
        v_global_vendor_name = v_local_vendor_name;
    }

    v_area_count = 0;


    // INPUT OUTPUT reg setting

    for(v_area_count = 0; v_global_slave_syncs[v_area_count].index != 0xff; ++v_area_count)
    {
        v_area_pdos_limit = v_global_slave_syncs[v_area_count].n_pdos;

#if ETHERCAT_SLAVE_CLASS_DEBUG
        d_print("[syncs]index : 0x%X\r\n",v_global_slave_syncs[v_area_count].index);
        d_print("[syncs]n_pdos : %d\r\n",v_area_pdos_limit);
#endif
        for(v_area_pdos_count = 0; v_area_pdos_count < v_area_pdos_limit; ++v_area_pdos_count)
        {
            v_area_ptr_pdos_temp = v_global_slave_syncs[v_area_count].pdos + v_area_pdos_count;
            v_area_entries_limit = (*v_area_ptr_pdos_temp).n_entries;

#if ETHERCAT_SLAVE_CLASS_DEBUG
            d_print("[pdos]index : 0x%X\r\n",(*v_area_ptr_pdos_temp).index);
            d_print("[pdos]n_entries : %d\r\n",(*v_area_ptr_pdos_temp).n_entries);
#endif
            for(v_area_entries_count = 0; v_area_entries_count < v_area_entries_limit; ++v_area_entries_count, ++v_area_total_entries_count)
            {
                v_area_ptr_etries_temp = v_area_ptr_pdos_temp->entries + v_area_entries_count;


                if(v_global_slave_syncs[v_area_count].dir == EC_DIR_OUTPUT)
                {
                    slave_domain_out_regs[v_area_inreg_count].alias = 0;
                    slave_domain_out_regs[v_area_inreg_count].position = v_local_slave_position;
                    slave_domain_out_regs[v_area_inreg_count].vendor_id = v_global_vendor_id;
                    slave_domain_out_regs[v_area_inreg_count].product_code = v_global_product_id;

                    slave_domain_out_regs[v_area_inreg_count].index = (*v_area_ptr_etries_temp).index;
                    slave_domain_out_regs[v_area_inreg_count].subindex = (*v_area_ptr_etries_temp).subindex;
                    slave_domain_out_regs[v_area_inreg_count].offset = (&v_global_offset_buffer[v_area_total_entries_count]);

#if ETHERCAT_SLAVE_CLASS_DEBUG
                    d_print("[DEBUG]f_init_initialize IO-reg [OUT]\r\n");
                    d_print("[DEBUG]Vendor ID : 0x%X\r\n",slave_domain_out_regs[v_area_inreg_count].vendor_id);
                    d_print("[DEBUG]Product ID : 0x%X\r\n",slave_domain_out_regs[v_area_inreg_count].product_code);
                    d_print("[DEBUG]index : 0x%X\r\n",slave_domain_out_regs[v_area_inreg_count].index);
                    d_print("[DEBUG]subindex : 0x%X\r\n",slave_domain_out_regs[v_area_inreg_count].subindex);
#endif
                    v_area_inreg_count++;

                }
                else if(v_global_slave_syncs[v_area_count].dir == EC_DIR_INPUT)
                {
                    slave_domain_in_regs[v_area_outreg_count].alias = 0;
                    slave_domain_in_regs[v_area_outreg_count].position = v_local_slave_position;
                    slave_domain_in_regs[v_area_outreg_count].vendor_id = v_global_vendor_id;
                    slave_domain_in_regs[v_area_outreg_count].product_code = v_global_product_id;

                    slave_domain_in_regs[v_area_outreg_count].index = (*v_area_ptr_etries_temp).index;
                    slave_domain_in_regs[v_area_outreg_count].subindex = (*v_area_ptr_etries_temp).subindex;
                    slave_domain_in_regs[v_area_outreg_count].offset = (&v_global_offset_buffer[v_area_total_entries_count]);


#if ETHERCAT_SLAVE_CLASS_DEBUG
                    d_print("[DEBUG]f_init_initialize IO-reg [IN]\r\n");
                    d_print("[DEBUG]Vendor ID : 0x%X\r\n",slave_domain_in_regs[v_area_outreg_count].vendor_id);
                    d_print("[DEBUG]Product ID : 0x%X\r\n",slave_domain_in_regs[v_area_outreg_count].product_code);
                    d_print("[DEBUG]index : 0x%X\r\n",slave_domain_in_regs[v_area_outreg_count].index);
                    d_print("[DEBUG]subindex : 0x%X\r\n",slave_domain_in_regs[v_area_outreg_count].subindex);
#endif
                    v_area_outreg_count++;
                }

            }
        }
    }
}


void ecatslave::f_init_master_insert(void * v_local_ecat_master)
{
    v_global_ecat_master_ptr = v_local_ecat_master;
}



/**
  * 2. EtherCAT slave oper part
  *
*/



void ecatslave::f_oper_send_process()
{
    long pos_step_val;
    uint8_t *v_area_domain1_pd = *((ecatmaster *)v_global_ecat_master_ptr)->f_get_domain1_pd_ptr();
    if (((ecatmaster *)v_global_ecat_master_ptr)->f_get_master_state_ptr()->al_states< EC_AL_STATES_OP
            ||((ecatmaster *)v_global_ecat_master_ptr)->f_get_domain_state_ptr()->state.wc_state != EC_WC_COMPLETE)
    {
        return ;
    }
    pos_step_val = v_global_run_speed;

    if ( v_global_req_servo_state == REQ_SERVO_RUN)
    {
        if (v_global_cur_step  == step_target_pos_fw)
        {
            v_global_set_target_val += pos_step_val;
            EC_WRITE_U32(v_area_domain1_pd + f_oper_get_offset_value(TARGET_PORSITION) , v_global_set_target_val);
//            d_print("[v_local_slv_inx:] = target_val:%ld\r\n", v_global_set_target_val);	//Print current position
        }
        else if (v_global_cur_step  == step_target_pos_rev)
        {
            v_global_set_target_val -= pos_step_val;
            EC_WRITE_U32(v_area_domain1_pd + f_oper_get_offset_value(TARGET_PORSITION), v_global_set_target_val);
//            d_print("[v_local_slv_inx:] = target_val:%ld\r\n", v_global_set_target_val);	//Print current position
        }


    }
}


void ecatslave::f_oper_recv_process()
{
    uint8_t *v_area_domain1_pd = *((ecatmaster *)v_global_ecat_master_ptr)->f_get_domain1_pd_ptr();

    long val;
    val = EC_READ_U16(v_area_domain1_pd + f_oper_get_offset_value(STATUS_WORD));
    if (val != v_global_cur_status_word)
        v_global_cur_status_word = val;

    val = EC_READ_S32(v_area_domain1_pd + f_oper_get_offset_value(PORSITION_VALUE));
    if (is_new_pos(val, v_global_cur_position))
       v_global_cur_position = val;

    val = EC_READ_S32(v_area_domain1_pd + f_oper_get_offset_value(TARGET_PORSITION));
    if (is_new_pos(val, v_global_tar_position))
       v_global_tar_position = val;

    val = EC_READ_S32(v_area_domain1_pd + f_oper_get_offset_value(TARGET_VELOCITY));
    if (is_new_pos(val, v_global_tar_velocity))
       v_global_tar_velocity = val;

    val = EC_READ_S16(v_area_domain1_pd + f_oper_get_offset_value(TARGET_TORQUE));
    if (is_new_pos(val, v_global_tar_torque))
       v_global_tar_torque = val;
}



unsigned int ecatslave::f_oper_get_offset_value(int v_local_index)
{
    int v_area_entries_count = 0;

    for(v_area_entries_count = 0; slave_domain_out_regs[v_area_entries_count].index != 0x00; ++v_area_entries_count)
    {
        if(slave_domain_out_regs[v_area_entries_count].index == v_local_index)
        {
            return REF(slave_domain_out_regs[v_area_entries_count].offset);
        }
    }


    for(v_area_entries_count = 0; slave_domain_in_regs[v_area_entries_count].index != 0x00; ++v_area_entries_count)
    {
        if(slave_domain_in_regs[v_area_entries_count].index == v_local_index)
        {
            return REF(slave_domain_in_regs[v_area_entries_count].offset);
        }
    }

    return 0;
}


