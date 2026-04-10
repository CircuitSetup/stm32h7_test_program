from pathlib import Path

Import("env")


project_dir = Path(env["PROJECT_DIR"])
build_dir = Path(env["PROJECT_BUILD_DIR"]) / env["PIOENV"] / "radio_sources"
libdeps_dir = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env["PIOENV"]

cyw43_root = project_dir / "third_party" / "cyw43-driver" / "src"
btstack_src_root = project_dir / "third_party" / "btstack" / "src"
btstack_platform_root = project_dir / "third_party" / "btstack" / "platform" / "embedded"
btstack_chipset_root = project_dir / "third_party" / "btstack" / "chipset"
lwip_root = libdeps_dir / "STM32duino LwIP" / "src"

env.Append(CPPPATH=[str(lwip_root)])

env.BuildSources(
    str(build_dir / "cyw43"),
    str(cyw43_root),
    src_filter=[
        "+<cyw43_ctrl.c>",
        "+<cyw43_ll.c>",
        "+<cyw43_lwip.c>",
        "+<cyw43_sdio.c>",
    ],
)

env.BuildSources(
    str(build_dir / "btstack_src"),
    str(btstack_src_root),
    src_filter=[
        "+<ad_parser.c>",
        "+<ble/att_db.c>",
        "+<ble/att_dispatch.c>",
        "+<ble/att_server.c>",
        "+<ble/gatt_client.c>",
        "+<btstack_linked_list.c>",
        "+<btstack_linked_queue.c>",
        "+<btstack_ltv_builder.c>",
        "+<btstack_memory.c>",
        "+<btstack_memory_pool.c>",
        "+<btstack_run_loop.c>",
        "+<btstack_tlv.c>",
        "+<btstack_tlv_none.c>",
        "+<btstack_util.c>",
        "+<hci.c>",
        "+<hci_cmd.c>",
        "+<hci_dump.c>",
        "+<hci_event.c>",
        "+<hci_event_builder.c>",
        "+<hci_transport_h4.c>",
        "+<l2cap.c>",
        "+<l2cap_signaling.c>",
    ],
)

env.BuildSources(
    str(build_dir / "btstack_platform"),
    str(btstack_platform_root),
    src_filter=[
        "+<btstack_run_loop_embedded.c>",
        "+<btstack_uart_block_embedded.c>",
    ],
)

env.BuildSources(
    str(build_dir / "btstack_chipset"),
    str(btstack_chipset_root),
    src_filter=[
        "+<bcm/btstack_chipset_bcm.c>",
    ],
)
