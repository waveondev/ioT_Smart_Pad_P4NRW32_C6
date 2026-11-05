idf_component_register(
    SRCS 
        "main.c"
        "gc2053_camera.c"
        "dummy_camera.c"
        "rtsp_server.c"
        "loadcell.c"
        "esp_wifi_p4_stub.c"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        nvs_flash 
        driver 
        sdmmc 
        esp_cam_sensor
        esp_sccb_intf
        esp_driver_cam
        esp_driver_isp 
        console 
        esp_event 
        esp_netif 
        lwip
        mdns
        espressif__esp_video
        espressif__esp_h264
        espressif__esp_hosted
        espressif__esp_wifi_remote
)

# ?뙚 留곸빱???곕뱶 肄붾뱶 ?쒓굅(--gc-sections)濡쒕????ㅽ뀅 ?щ낵 媛뺤젣 蹂댁〈
target_link_options(${COMPONENT_LIB} INTERFACE
    "-Wl,--undefined=esp_wifi_stop"
    "-Wl,--undefined=esp_wifi_internal_set_sta_ip"
    "-Wl,--undefined=WIFI_EVENT"
)

# ?뙚 RPC ?쒕툕 ?쇱씠釉뚮윭由?吏곸젒 紐낆떆
if(TARGET eh_host_feat_rpc)
    target_link_libraries(${COMPONENT_LIB} PRIVATE eh_host_feat_rpc)
endif()
if(TARGET eh_host_feat_rpc_ext_v2)
    target_link_libraries(${COMPONENT_LIB} PRIVATE eh_host_feat_rpc_ext_v2)
endif()

# ?뙚 諛붿씠?덈━ ?곗씠???꾨쿋??(network_adapter.bin 議댁옱 ?щ? ?뺤씤 ???곌껐)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/network_adapter.bin")
    target_add_binary_data(${COMPONENT_LIB} "network_adapter.bin" BINARY)
endif()

target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__esp_video/private_include")

