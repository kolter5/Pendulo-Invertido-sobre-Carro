# -*- coding: utf-8 -*-
"""
Grava a saida serial do Arduino em um arquivo CSV.

O monitor serial da IDE nao salva em arquivo; este script faz isso. Descarta
as linhas de comentario (que comecam com '#'), reescreve o mostrador de
posicionamento sempre na mesma linha do terminal, e encerra sozinho quando o
sketch imprime "# fim do ensaio".

Duas formas de usar:

  1. Dois cliques no gravar_ensaio.bat: pergunta a porta e o arquivo, e a
     janela so fecha quando voce mandar.

  2. Pela linha de comando, ja passando tudo:
         python grava_serial.py COM3 ensaio_soltura.csv

Interromper antes do fim: Ctrl+C (o que ja foi gravado e mantido).

Este e o unico script Python que sobrou: as analises estao todas em Octave.
Ele fica em Python porque precisa falar com a porta serial, o que o Octave
so faz com pacote extra.
"""

import os
import sys
import time

import serial
import serial.tools.list_ports

AQUI = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(AQUI)          # pasta verificacao/
BAUD = 115200

# Sem os dois argumentos, foi aberto com duplo clique: pergunta o que falta
# e segura a janela no final.
INTERATIVO = len(sys.argv) < 3


def pausar():
    if INTERATIVO:
        try:
            input("\nPressione Enter para fechar...")
        except (EOFError, KeyboardInterrupt):
            pass


def ler(prompt):
    """input() sem espacos nem BOM (que aparece quando a entrada vem de pipe)."""
    return input(prompt).strip().lstrip("﻿").strip()


def escolher_porta():
    portas = list(serial.tools.list_ports.comports())
    if not portas:
        print("Nenhuma porta serial encontrada.")
        print("  - o Arduino esta conectado pelo cabo USB?")
        print("  - o monitor serial da IDE esta FECHADO?")
        return None
    print("Portas seriais disponiveis:")
    for i, p in enumerate(portas, 1):
        print("  [{}] {:<8s} {}".format(i, p.device, p.description))
    if len(portas) == 1:
        print("\nUsando {} (unica disponivel).".format(portas[0].device))
        return portas[0].device
    while True:
        r = ler("\nNumero da porta (ou o nome, ex. COM3): ")
        if not r:
            return None
        if r.isdigit() and 1 <= int(r) <= len(portas):
            return portas[int(r) - 1].device
        if any(r.upper() == p.device.upper() for p in portas):
            return r.upper()
        print("Opcao invalida.")


def escolher_arquivo():
    print("\nQual ensaio esta rodando?")
    print("  [1] soltura de angulo controlado -> ensaio_soltura.csv")
    print("  [2] angulo, registro simples     -> ensaio_angulo.csv")
    print("  [3] atuador                      -> ensaio_atuador.csv")
    print("  [4] posicao do carro             -> ensaio_posicao.csv")
    r = ler("Numero (ou digite outro nome de arquivo): ")
    padrao = {"": "ensaio_soltura.csv", "1": "ensaio_soltura.csv",
              "2": "ensaio_angulo.csv", "3": "ensaio_atuador.csv",
              "4": "ensaio_posicao.csv"}
    if r in padrao:
        return padrao[r]
    return r if r.lower().endswith(".csv") else r + ".csv"


def main():
    if INTERATIVO:
        print("=" * 62)
        print("GRAVADOR DE ENSAIO - TCC pendulo invertido")
        print("=" * 62 + "\n")
        porta = escolher_porta()
        if porta is None:
            return
        destino = escolher_arquivo()
    else:
        porta = sys.argv[1]
        destino = sys.argv[2]

    if not os.path.isabs(destino):
        destino = os.path.join(BASE, "dados", destino)
    os.makedirs(os.path.dirname(destino), exist_ok=True)

    if os.path.exists(destino):
        r = input("\n'{}' ja existe. Sobrescrever? [s/N] "
                  .format(os.path.basename(destino))).strip().lower()
        if r != "s":
            print("Cancelado - nada foi alterado.")
            return

    print("\nPorta   :", porta)
    print("Arquivo :", destino)
    print("Feche o monitor serial da IDE antes de continuar.\n")

    n = 0
    em_linha = [False]     # True quando a ultima saida foi reescrita com \r

    def fixa(texto):
        print("\r  {:<72s}".format(texto), end="", flush=True)
        em_linha[0] = True

    def rola(texto):
        if em_linha[0]:
            print()
            em_linha[0] = False
        print(" ", texto)

    with serial.Serial(porta, BAUD, timeout=2) as ser, \
            open(destino, "w", encoding="utf-8", newline="") as f:
        # Abrir a porta reseta o Arduino. Espera o bootloader e o setup()
        # imprimirem, mostra o que veio, e so entao pede o comando de inicio.
        time.sleep(2.5)
        while ser.in_waiting:
            linha = ser.readline().decode("utf-8", errors="replace").strip()
            if linha:
                rola(linha)

        print("\n" + "=" * 62)
        print("  Posicione a bancada e pressione ENTER para INICIAR.")
        print("=" * 62)
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            pass
        ser.write(b"S\n")
        print("iniciado. (Ctrl+C encerra)\n")

        try:
            while True:
                linha = ser.readline().decode("utf-8", errors="replace").strip()
                if not linha:
                    continue
                if linha.startswith("#"):
                    # "# pos: ..." e o mostrador de posicionamento, que se
                    # repete varias vezes por segundo: fica fixo na mesma linha
                    if linha.startswith("# pos:"):
                        fixa(linha[2:])
                    else:
                        rola(linha)
                    if "fim do ensaio" in linha:
                        break
                    continue
                f.write(linha + "\n")
                n += 1
                if n % 50 == 0:
                    fixa("{} amostras gravadas".format(n))
        except KeyboardInterrupt:
            rola("interrompido pelo usuario")

    if em_linha[0]:
        print()
    print("\n{} amostras gravadas em {}".format(n, destino))
    if n < 50:
        print("AVISO: pouquissimas amostras. Verifique se o sketch correto")
        print("       esta gravado e se o monitor serial da IDE estava fechado.")
    else:
        print("Agora rode a analise no Octave.")


try:
    main()
except serial.SerialException as e:
    print("\nERRO ao abrir a porta serial:")
    print(" ", e)
    print("\nCausa mais comum: o monitor serial da IDE do Arduino esta aberto")
    print("e segurando a porta. Feche-o e tente de novo.")
except Exception as e:                      # noqa: BLE001 - janela nao pode sumir
    print("\nERRO inesperado: {}: {}".format(type(e).__name__, e))
pausar()
