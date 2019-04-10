void lowpan_configuration() {
  char buffer[256];
  int id = 0;
  printf("Starting WPAN configuration\n");
  printf("Type C to be coordinator\nType N to be node\n\n\n");

  scanf("%1s", buffer);
  system("ifdown wpan0");
  if (strcmp("c", buffer) == 0) {
    // Set coordinator
    system("i8sak wpan0 startpan cd:ab");
    system("i8sak set chan 26");
    sleep(1);
    printf("Choose your ID (00 to FF)\n\n");
    scanf("%s", buffer);
    if (strlen(buffer) < 2) {
      printf("Error ID must be between 00 and FF\n");
      return 0;
    }
    system("i8sak set saddr 42:01");
    sprintf(buffer, "i8sak set eaddr 00:fa:de:00:de:ad:be:%c%c", buffer[0],
            buffer[1]);
    printf("Your hardware address is: %s\n\n", buffer);
    system(buffer);
    system("i8sak acceptassoc");

  } else if (strcmp("n", buffer) == 0) {
    // Set node
    system("i8sak wpan0");
    system("i8sak set chan 26");
    system("i8sak set panid cd:ab");
    sleep(1);
    printf("Choose your ID (00 to FF)\n\n");
    scanf("%s", buffer);
    if (strlen(buffer) < 2) {
      printf("Error ID must be between 00 and FF\n");
      return 0;
    }

    sprintf(buffer, "i8sak set eaddr 00:fa:de:00:de:ad:be:%c%c", buffer[0],
            buffer[1]);
    system(buffer);
    system("i8sak set ep_saddr 42:01");
    system("i8sak set saddr 42:02");
    printf("Your hardware address is: %s\n\n", buffer);
    system("i8sak assoc");
  } else {
    printf("Wrong profile\r\n");
  }
  system("ifup wpan0");

  printf("Mounting proc file system\n");
  system("mount -t procfs /proc");
  system("cat proc/net/wpan0");
  printf("\n\n");
}
