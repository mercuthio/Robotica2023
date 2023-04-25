#!/bin/bash

directorio_remoto="Robotica_test_files/trabajo/"
ip_remota="10.1.31.226"
usuario="pi"


# Si me pasan por parámetro la opción --subir, subo los archivos al robot
if [ "$1" = "--subir" ]; then
    echo "Subiendo archivos al robot..."
    scp -r * ${usuario}@${ip_remota}:${directorio_remoto}
    echo "Archivos subidos"
fi

# Si me pasan por parámetro la opción --bajar, descargo los archivos del robot
if [ "$1" = "--bajar" ]; then
    echo "Descargando archivos del robot..."
    scp -r ${usuario}@${ip_remota}:${directorio_remoto} .
    echo "Archivos descargados"
fi

# instrucción="ssh ${usuario}@${ip_remota} \"cd ${directorio_remoto}; python3 p2_base.py\""

# Si me pasan por parámetro la opción --ejecutar, ejecuto el programa en el robot
if [ "$1" = "--ejecutar" ]; then
    echo "Ejecutando programa en el robot..."
    # eval $instrucción
    ssh ${usuario}@${ip_remota} "cd ${directorio_remoto}; python3 p2_base.py"
    echo "Programa ejecutado"
fi
