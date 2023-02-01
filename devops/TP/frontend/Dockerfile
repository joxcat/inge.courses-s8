FROM httpd:2.2-alpine
COPY ./index.html /usr/local/apache2/htdocs/
RUN echo "Include /etc/apache2/vhosts.d/*.conf" >> /usr/local/apache2/conf/httpd.conf 
COPY ./vhost.conf /etc/apache2/vhosts.d/


