
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle
from datetime import datetime

def generar_reporte_pdf(
    tiempo_malo, rpm_malo, u_malo,
    tiempo_bueno, rpm_bueno, u_bueno
):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    nombre_archivo = f"Reporte_Motores_{timestamp}.pdf"

    doc = SimpleDocTemplate(nombre_archivo, pagesize=A4)
    styles = getSampleStyleSheet()
    elements = []

    # --- Título
    title_style = ParagraphStyle(
        'TitleStyle',
        parent=styles['Heading1'],
        alignment=1,
        textColor=colors.HexColor("#003366"),
        fontSize=18,
        spaceAfter=20
    )
    elements.append(Paragraph("📊 Reporte de Simulación de Motores", title_style))

    # --- Subtítulo
    subtitle = f"Generado el {datetime.now().strftime('%d/%m/%Y %H:%M:%S')}"
    elements.append(Paragraph(subtitle, styles['Normal']))
    elements.append(Spacer(1, 20))

    # --- Sección Motor Malo
    elements.append(Paragraph("Motor con Fallo", styles['Heading2']))
    data_malo = [["Tiempo (s)", "RPM", "Entrada u"]] + [
        [f"{t:.2f}", f"{rpm:.2f}", f"{u:.2f}"]
        for t, rpm, u in zip(tiempo_malo[:20], rpm_malo[:20], u_malo[:20])  # primeras 20 filas
    ]

    tabla_malo = Table(data_malo)
    tabla_malo.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#b71c1c")),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN',(0,0),(-1,-1),'CENTER'),
        ('INNERGRID', (0,0), (-1,-1), 0.25, colors.gray),
        ('BOX', (0,0), (-1,-1), 0.25, colors.black)
    ]))
    elements.append(tabla_malo)
    elements.append(Spacer(1, 20))

    # --- Sección Motor Bueno
    elements.append(Paragraph("Motor en Buen Estado", styles['Heading2']))
    data_bueno = [["Tiempo (s)", "RPM", "Entrada u"]] + [
        [f"{t:.2f}", f"{rpm:.2f}", f"{u:.2f}"]
        for t, rpm, u in zip(tiempo_bueno[:20], rpm_bueno[:20], u_bueno[:20])
    ]

    tabla_bueno = Table(data_bueno)
    tabla_bueno.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#1b5e20")),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN',(0,0),(-1,-1),'CENTER'),
        ('INNERGRID', (0,0), (-1,-1), 0.25, colors.gray),
        ('BOX', (0,0), (-1,-1), 0.25, colors.black)
    ]))
    elements.append(tabla_bueno)
    elements.append(Spacer(1, 20))

    # --- Firma / Pie
    elements.append(Paragraph("<b>Reporte generado automáticamente por el sistema PID Control.</b>", styles['Normal']))

    # --- Crear PDF
    doc.build(elements)
    print(f"📁 Reporte PDF guardado como: {nombre_archivo}")
