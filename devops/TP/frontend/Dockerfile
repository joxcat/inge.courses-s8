FROM httpd:2.4
RUN sed -i \
		-e 's/^#\(LoadModule .*mod_proxy.so\)/\1/' \
		-e 's/^#\(LoadModule .*mod_proxy_http.so\)/\1/' \
		-e 's/^#\(LoadModule .*mod_proxy_ajp.so\)/\1/' \
		-e 's/^#\(LoadModule .*mod_proxy_balancer.so\)/\1/' \
    -e 's/^#\(LoadModule .*mod_slotmem_shm.so\)/\1/' \
    -e 's/^#\(LoadModule .*mod_lbmethod_byrequests\)/\1/' \
		conf/httpd.conf

COPY ./index.html /usr/local/apache2/htdocs/
RUN echo "Include /etc/apache2/vhosts.d/*.conf" >> /usr/local/apache2/conf/httpd.conf 
COPY ./vhost.conf /etc/apache2/vhosts.d/


