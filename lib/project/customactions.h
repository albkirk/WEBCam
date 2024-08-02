// Function to insert customized MQTT Configuration actions
void custom_mqtt(String command, String cmd_value) {
    if ( command == "BckpRstr" ) {
        if (cmd_value == "") telnet_println("Restore data is empty");
        else {
            DeserializationError error = deserializeJson(config_doc, cmd_value); //Deserialize string to JSON data
            if (error) {telnet_print("JSON DeSerialization failure: "); telnet_println(error.f_str());}
            else {
                strcpy(config.DeviceName, config_doc["DeviceName"]);
                strcpy(config.Location, config_doc["Location"]);
/*
                config.LOWER_LEVEL =    config_doc["LOWER_Pos"];
                config.UPPER_LEVEL =    config_doc["UPPER_Pos"];
*/
                storage_write();
                bckp_rstr_flag = true;
                telnet_println("BckpRstr with success");
            }
        }
    }

//  if ( command == "send_Telemetry" && bool(cmd_value.toInt())) { gps_update(); print_gps_data(); send_Telemetry(); }

    if ( command == "Light") {
        if ( Light_Last == bool(cmd_value.toInt()) ) mqtt_publish(mqtt_pathtele, "Light", String(Light));
        else Light = bool(cmd_value.toInt());
    }
    if ( command == "OnAir") {
        if ( OnAir_Last == bool(cmd_value.toInt()) ) mqtt_publish(mqtt_pathtele, "OnAir", String(OnAir));
        else OnAir = bool(cmd_value.toInt());
    }
    if ( command == "VFlip") {
        VFlip = bool(cmd_value.toInt());
        cam_sensor = esp_camera_sensor_get();
        if (OnAir) cam_sensor->set_vflip(cam_sensor, VFlip ? 1 : 0);
        mqtt_publish(mqtt_pathtele, "VFlip", String(VFlip));
    }
    if ( command == "HMirror") {
        HMirror = bool(cmd_value.toInt());
        cam_sensor = esp_camera_sensor_get();
        if (OnAir) cam_sensor->set_hmirror(cam_sensor, HMirror ? 1 : 0);
        mqtt_publish(mqtt_pathtele, "HMirror", String(HMirror));
    }

}

void custom_update(){
    //yield();
    //ambient_data();
    //mqtt_dump_data(mqtt_pathtele, "Telemetry");
    mqtt_publish(mqtt_pathtele, "Light", String(Light));
    //mqtt_publish(mqtt_pathtele, "OnAir", String(OnAir));
    //mqtt_publish(mqtt_pathtele, "Public_IP", public_ip());
    telnet_print("OnAir: " + String(OnAir));
    telnet_print(" - VFlip: " + String(VFlip));
    telnet_print(" - HMirror: " + String(HMirror));
    telnet_print(" - Stream_VIDEO: " + String(Stream_VIDEO));
    telnet_println(" - Stream_AUDIO: " + String(Stream_AUDIO));
}
