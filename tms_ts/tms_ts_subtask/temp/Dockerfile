FROM mongo:latest

RUN apt-get update && apt-get install -y unzip

COPY ./dump.zip /dump.zip
RUN unzip /dump.zip -d /tmp

EXPOSE 27017

CMD mongod --fork --logpath /var/log/mongodb.log; \
    mongorestore /tmp/dump/; \
    mongod --shutdown; \