// Your WiFi credentials
const char ssid[] = "my_wifi";
const char password[] =  "my_wifi_password";

// RTK2Go works well and is free
const char casterHost[] = "rtk2go.com"; 
const uint16_t casterPort = 2101;
const char casterUser[] = "my_email@company.com"; // User must provide their own email address to use RTK2Go
const char casterUserPW[] = "";
const char mountPoint[] = "bldr_SparkFun1"; // The mount point you want to get data from

// Emlid Caster also works well and is free
// const char casterHost[] = "caster.emlid.com";
// const uint16_t casterPort = 2101;
// const char casterUser[] = "u99696"; // User name and pw must be obtained through their web portal
// const char casterUserPW[] = "466zez";
// const char mountPoint[] = "MP1979"; // The mount point you want to get data from