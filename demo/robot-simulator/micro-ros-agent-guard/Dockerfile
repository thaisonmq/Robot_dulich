FROM microros/micro-ros-agent:humble

COPY guarded-entrypoint.sh /opt/rovera/guarded-entrypoint.sh
RUN chmod 0555 /opt/rovera/guarded-entrypoint.sh

ENTRYPOINT ["/opt/rovera/guarded-entrypoint.sh"]
CMD ["--help"]
