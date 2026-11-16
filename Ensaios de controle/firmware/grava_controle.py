# -*- coding: utf-8 -*-
"""Grava os ensaios de controle e resume o resultado na hora.

O arquivo e escrito LINHA A LINHA, desde a primeira amostra. Se a janela for
fechada, se faltar energia ou se a haste cair, o que ja veio esta no disco.

Uso:
    python grava_controle.py COM3 e1_condicao_inicial.csv 30

Sem argumentos, pergunta. Ctrl+C encerra e mostra o resumo.
Formato gravado: t,ref,theta,theta_p,x,x_p,xi,u,pwm
"""

import os
import sys
import time

import serial
import serial.tools.list_ports

AQUI = os.path.dirname(os.path.abspath(__file__))
BAUD = 115200
CABECALHO = "t,ref,theta,theta_p,x,x_p,xi,u,d,pwm"


def escolhe_porta():
    portas = list(serial.tools.list_ports.comports())
    if not portas:
        print("nenhuma porta serial encontrada")
        sys.exit(1)
    if len(portas) == 1:
        print("porta: %s (%s)" % (portas[0].device, portas[0].description))
        return portas[0].device
    for i, p in enumerate(portas):
        print("  %d) %s  %s" % (i + 1, p.device, p.description))
    return portas[int(input("qual? ")) - 1].device


def resume(caminho):
    """Le o arquivo gravado e imprime o que o Capitulo 5 precisa."""
    dados = []
    with open(caminho, encoding="utf-8") as f:
        for l in f:
            c = l.strip().split(",")
            if len(c) != 10:
                continue
            try:
                dados.append([float(v) for v in c])
            except ValueError:
                continue

    print("\n" + "-" * 58)
    if len(dados) < 20:
        print("  so %d amostras: a corrida nao chegou a armar" % len(dados))
        print("  o arquivo foi criado assim mesmo, com o que veio")
        return

    t = [d[0] for d in dados]
    ref = [d[1] for d in dados]
    th = [d[2] * 180 / 3.14159265 for d in dados]
    x = [d[4] * 1000 for d in dados]
    u = [d[7] for d in dados]

    print("  duracao             : %.1f s   (%d amostras)" % (t[-1] - t[0], len(dados)))
    print("  |theta| maximo      : %.2f graus" % max(abs(v) for v in th))
    print("  theta rms           : %.3f graus" % ((sum(v * v for v in th) / len(th)) ** 0.5))
    print("  tensao maxima       : %.2f V         (simulacao: 6,0 V)" % max(abs(v) for v in u))
    print("  tensao rms          : %.2f V" % ((sum(v * v for v in u) / len(u)) ** 0.5))
    print("  curso usado         : %.0f mm" % (max(x) - min(x)))

    i = next((k for k in range(1, len(ref)) if abs(ref[k] - ref[k - 1]) > 1e-4), None)
    if i:
        alvo = ref[-1] * 1000
        faixa = 0.02 * abs(alvo - ref[i - 1] * 1000)
        fora = [k for k in range(i, len(x)) if abs(x[k] - alvo) > faixa]
        if fora and fora[-1] < len(x) - 5:
            print("  degrau em t = %.1f s, acomodou em %.1f s   (simulacao: 4,5 s)"
                  % (t[i], t[fora[-1]] - t[i]))
        else:
            print("  degrau em t = %.1f s, nao acomodou no registro" % t[i])
        print("  inclinacao no degrau: %.2f graus      (simulacao: 0,24)"
              % max(abs(v) for v in th[i:]))
        print("  erro final          : %.1f mm" % (x[-1] - alvo))

    amax = max(abs(v) for v in th)
    umax = max(abs(v) for v in u)
    print()
    if amax > 10:
        print("  a haste passou de 10 graus: caiu no fim da corrida")
    elif umax > 9.5:
        print("  a tensao chegou perto da saturacao")
    else:
        print("  corrida limpa")


def main():
    porta = sys.argv[1] if len(sys.argv) > 1 else escolhe_porta()
    nome = sys.argv[2] if len(sys.argv) > 2 else input("arquivo: ").strip()
    limite = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    if not nome.endswith(".csv"):
        nome += ".csv"
    destino = nome if os.path.isabs(nome) else os.path.join(AQUI, nome)

    print("\ngravando em:\n  %s\n" % destino)
    saida = open(destino, "w", encoding="utf-8", newline="\n")
    saida.write(CABECALHO + "\n")
    saida.flush()

    # As linhas de # que o firmware imprime na partida dizem qual binario esta
    # rodando. Elas nao cabem no CSV, que as rotinas do Octave leem como numero,
    # e por isso vao para um arquivo ao lado.
    marca = os.path.splitext(destino)[0] + ".cabecalho.txt"
    cabecalho = []

    n = 0
    abortou = False
    try:
        with serial.Serial(porta, BAUD, timeout=1) as s:
            # Limpa o lixo de antes do reset ANTES de esperar, senao as linhas
            # que o setup() imprime na partida, que dizem qual binario esta
            # rodando, sao apagadas junto.
            s.reset_input_buffer()
            time.sleep(0.2)
            print("porta aberta. Leve a haste a vertical para armar.\n")

            t0 = None
            ultimo = 0.0
            while True:
                bruto = s.readline().decode("utf-8", "replace").strip()
                if not bruto:
                    continue

                if bruto.startswith("#"):
                    if "abort" in bruto:
                        abortou = True
                        print("\n  >>> abortado pelo firmware")
                    elif t0 is None:
                        cabecalho.append(bruto)
                        sys.stdout.write("\r  " + bruto[1:].strip() + "        ")
                        sys.stdout.flush()
                    continue

                if bruto.startswith("t,ref"):
                    continue

                c = bruto.split(",")
                if len(c) != 10:
                    continue

                saida.write(bruto + "\n")     # grava imediatamente
                n += 1
                if n % 10 == 0:
                    saida.flush()             # e descarrega a cada 10 linhas

                if t0 is None:
                    t0 = time.time()
                    print("armou, gravando\n")

                agora = time.time()
                if agora - ultimo > 0.2:
                    ultimo = agora
                    try:
                        sys.stdout.write("\r  t %5.1f s   theta %+6.2f graus   "
                                         "x %+7.1f mm   u %+6.2f V   " %
                                         (float(c[0]), float(c[2]) * 57.2958,
                                          float(c[4]) * 1000, float(c[7])))
                        sys.stdout.flush()
                    except ValueError:
                        pass

                if limite > 0 and (agora - t0) > limite:
                    print("\n\ntempo esgotado")
                    break
    except KeyboardInterrupt:
        print("\n\ninterrompido")
    except serial.SerialException as e:
        print("\n\nerro na porta: %s" % e)
    finally:
        saida.close()
        if cabecalho:
            with open(marca, "w", encoding="utf-8", newline="\n") as f:
                f.write("\n".join(cabecalho) + "\n")

    print("\narquivo fechado com %d linhas de dado" % n)
    if cabecalho:
        print("cabecalho do firmware guardado em %s" % os.path.basename(marca))
        for l in cabecalho:
            print("   %s" % l)
    else:
        print("ATENCAO: o firmware nao imprimiu cabecalho; o ensaio fica sem")
        print("identificacao de qual binario rodou.")
    if abortou:
        print("ATENCAO: o firmware abortou durante a corrida")
    resume(destino)


if __name__ == "__main__":
    main()
