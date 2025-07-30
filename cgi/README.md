### CGI Application Source Codes (.cgi)
this directory contain cgi application's source codes that implements with fcgi & nginx protocol

when configure platform_obsever, following cgis are required :

    config.cgi 
    (for ctrl rpi camera settings)

    capture.cgi
    (capture platform_observer's view and send to client)
    
    roi-setup.cgi
    (setup roi data for platform observing)

    stop-status.cgi
    (for get status of bus stop platform)

when configure rpi_server, following cgis are required :


    config.cgi 
    (for ctrl rpi camera settings)

    sequence.cgi 
    (for get bus line number sequence)

    sleep.cgi 
    (for ctrl STM32 board sleep setup)

    line-info.cgi 
    (for get line number from license plante number)


### Installation
we use make for build & install applications to nginx directories

```bash
# install config.cgi
make install-cgi target=config
```

when installation in completed you can see
```
sudo cp build/config.cgi /usr/lib/cgi-bin/config.cgi
sudo chmod +x /usr/lib/cgi-bin/config.cgi
```
this make you build from sources and install cgis to `/usr/lib/cgi-bin/`



